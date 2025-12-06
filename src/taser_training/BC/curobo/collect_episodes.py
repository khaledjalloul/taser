import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument(
    "--num_plans",
    type=int,
    default=10_000,
    help="Total number of plans to store. Default: infinite",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate"
)
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

###############################################################

import sys
from datetime import datetime
from pathlib import Path

import isaacsim.core.utils.stage as stage_utils
import numpy as np
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.cloner import GridCloner
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationActions
from pxr import UsdGeom  # type: ignore
from tqdm import tqdm

from taser.common.datatypes import TaserJointState
from taser.common.model import USD_PATH
from taser_training.BC.curobo.curobo_planner import CuroboPlanner
from taser_training.BC.utils.dataset import GPTEpisode, save_episode_to_h5

Kp = 3.0
Kd = 0.2


class CuroboDatasetCollector:
    def __init__(self):
        self.num_envs: int = args.num_envs
        self.num_plans_to_save: int = args.num_plans
        self.num_plans_saved: int = 0

        self.world: World = World()
        self.robot = self._spawn_robot()
        self.env_origins = self.robot.get_world_poses()[0]
        self.world.initialize_physics()

        self.planner = CuroboPlanner(
            scene=self.world.scene,
            robot=self.robot,
            interpolation_dt=self.world.get_physics_dt(),
            env_origins=self.env_origins,
            is_static_terrain=True,
        )

        self.pbar = tqdm(
            total=self.num_plans_to_save,
            desc="Generating Plans",
            file=sys.stdout,
            disable=False,
        )

        run_name = f"GPT_dataset_{datetime.now().strftime('%m%d_%H%M%S')}.h5"
        self.output_path = Path.cwd() / "outputs" / "BC" / "curobo_dataset" / run_name
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.needs_reset = False
        self.first_step = True
        self.world.add_physics_callback("taser_step", callback_fn=self.on_physics_step)

    def _spawn_robot(self):
        env_zero_path = "/World/envs/env_0"
        add_reference_to_stage(
            usd_path=str(USD_PATH), prim_path=f"{env_zero_path}/taser"
        )

        # Clone the environment
        cloner = GridCloner(spacing=2.0)
        cloner.define_base_env(env_zero_path)
        UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        cloner.clone(
            source_prim_path=env_zero_path,
            prim_paths=cloner.generate_paths("/World/envs/env", self.num_envs),
        )

        robot = Articulation(
            name="taser",
            prim_paths_expr="/World/envs/env.*/taser",
            # positions=self.env_origins,
        )

        # TODO: Enable self-collisions
        robot.set_enabled_self_collisions(np.full((self.num_envs), False))
        self.world.scene.add(robot)
        self.world.scene.add_default_ground_plane(z_position=-0.65)
        return robot

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self.first_step = False
        elif self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False
            self.first_step = True

    def run(self) -> None:
        while simulation_app.is_running():
            self.run_step()

            if self.num_plans_saved >= self.num_plans_to_save:
                break

    def run_step(self) -> None:
        self.pbar.set_postfix_str(
            f"Generating plans for {self.num_envs} envs | Total Saved: {self.num_plans_saved}",
        )

        episode = self.planner.plan()
        if episode is None:
            return

        dof_ids = TaserJointState.isaac_indices
        arm_dof_ids = np.hstack([dof_ids.left_arm, dof_ids.right_arm])
        num_arm_dof = len(arm_dof_ids)
        num_total_dof = self.robot.num_dof

        # Store start configurations in order to execute them in parallel
        start_cfgs = np.zeros((self.num_envs, num_total_dof))
        start_cfgs[:, dof_ids.left_arm] = episode.start_cfg[:, 0, :]
        start_cfgs[:, dof_ids.right_arm] = episode.start_cfg[:, 1, :]
        for env_idx in range(self.num_envs):
            VisualCuboid(
                prim_path=f"/World/targets/target_left{env_idx}",
                position=episode.target_position[env_idx, 0]
                + self.env_origins[env_idx],
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.63, 0.0, 0.8]),
            )
            VisualCuboid(
                prim_path=f"/World/targets/target_right{env_idx}",
                position=episode.target_position[env_idx, 1]
                + self.env_origins[env_idx],
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.0, 0.63, 0.8]),
            )

        # Prepare to store the executed plans in dataset format
        dataset_episodes = [
            GPTEpisode(
                joint_positions=np.empty((0, num_arm_dof)),
                joint_velocities=np.empty((0, num_arm_dof)),
                eef_positions=np.empty((0, 6)),
                target_positions=np.empty((0, 6)),
                actions=np.empty((0, num_arm_dof)),
            )
            for _ in range(self.num_envs)
        ]

        self.robot.set_world_poses(
            self.env_origins,
            np.array([1.0, 0.0, 0.0, 0.0])[None, :].repeat(self.num_envs, axis=0),
        )

        # Initialize the robot envs at the start configurations of their current plans
        self.robot.set_joint_positions(start_cfgs)
        self.robot.set_joint_velocities(np.zeros((self.num_envs, num_total_dof)))
        self.world.step(render=True)

        # Execute the plans until all envs are done
        step = 0
        max_step = episode.joint_velocities.shape[1]
        while step < max_step:
            actions = np.zeros((self.num_envs, num_total_dof))
            actions[:, TaserJointState.isaac_indices.locks] = [-0.5, 7.0, -0.5, 7.0]

            current_joint_pos = self.robot.get_joint_positions().copy()
            current_joint_vel = self.robot.get_joint_velocities().copy()

            # Apply the joint velocity at the current step of the plan or zero if the plan has finished
            dq_ref = episode.joint_velocities.reshape(self.num_envs, -1, num_arm_dof)[
                :, step
            ]
            q_ref = episode.joint_positions.reshape(self.num_envs, -1, num_arm_dof)[
                :, step
            ]

            # Add extra PD control for better tracking
            pos_err = q_ref - current_joint_pos[:, arm_dof_ids]
            vel_err = dq_ref - current_joint_vel[:, arm_dof_ids]

            actions[:, arm_dof_ids] = dq_ref + Kp * (pos_err) + Kd * (vel_err)

            for env_idx in range(self.num_envs):
                if step >= np.count_nonzero(episode.mask[env_idx]):
                    continue
                dataset_episodes[env_idx].joint_positions = np.vstack(
                    [
                        dataset_episodes[env_idx].joint_positions,
                        current_joint_pos[env_idx, arm_dof_ids],
                    ]
                )
                dataset_episodes[env_idx].joint_velocities = np.vstack(
                    [
                        dataset_episodes[env_idx].joint_velocities,
                        current_joint_vel[env_idx, arm_dof_ids],
                    ]
                )
                dataset_episodes[env_idx].target_positions = np.vstack(
                    [
                        dataset_episodes[env_idx].target_positions,
                        episode.target_position[env_idx].flatten(),
                    ]
                )
                dataset_episodes[env_idx].eef_positions = np.vstack(
                    [
                        dataset_episodes[env_idx].eef_positions,
                        episode.eef_positions[env_idx].reshape(-1, 6),
                    ]
                )
                dataset_episodes[env_idx].actions = np.vstack(
                    [
                        dataset_episodes[env_idx].actions,
                        actions[env_idx, arm_dof_ids],
                    ]
                )

            self.robot.apply_action(ArticulationActions(joint_velocities=actions))
            self.world.step(render=True)
            step += 1

        for env_episode in dataset_episodes:
            if env_episode.joint_positions.shape[0] == 0:
                continue  # Skip unsuccessful episodes

            save_episode_to_h5(
                episode=env_episode,
                file_path=self.output_path,
                episode_id=str(self.num_plans_saved),
            )
            self.num_plans_saved += 1
            self.pbar.update(1)


def main():
    sim = CuroboDatasetCollector()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
