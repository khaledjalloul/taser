import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument(
    "--num_plans",
    type=int,
    default=100_000,
    help="Total number of plans to store. Default: infinite",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate"
)
parser.add_argument(
    "--output_path",
    type=str,
    help="Output file path for the collected dataset. An existing file will be appended.",
)
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

###############################################################

import sys
from datetime import datetime
from pathlib import Path

import h5py
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
from taser_training.BC.curobo.curobo_planner import CuroboEpisode, CuroboPlanner
from taser_training.BC.utils.dataset import GPTEpisode

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

        self.dof_ids = TaserJointState.isaac_indices
        self.arm_dof_ids = np.hstack([self.dof_ids.left_arm, self.dof_ids.right_arm])
        self.num_arm_dof = len(self.arm_dof_ids)
        self.num_total_dof = self.robot.num_dof

        if args.output_path:
            self.output_path = Path(args.output_path)
        else:
            run_name = f"GPT_dataset_{datetime.now().strftime('%m%d_%H%M%S')}.h5"
            self.output_path = Path.cwd() / "outputs" / "BC" / "datasets" / run_name
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        if self.output_path.exists():
            with h5py.File(self.output_path, "r") as f:
                if "episodes" in f:
                    self.num_plans_saved = len(f["episodes"])
                    print(
                        f"Found {self.num_plans_saved} existing plans in {self.output_path}"
                    )

        self.pbar = tqdm(
            total=self.num_plans_to_save,
            initial=self.num_plans_saved,
            desc="Generating Plans",
            file=sys.stdout,
            disable=False,
        )

        self.needs_reset = True

    def setup(self) -> None:
        self.world.add_physics_callback(
            "taser_curobo_step", callback_fn=self.on_physics_step
        )

    def _spawn_robot(self) -> Articulation:
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

        robot = Articulation(name="taser", prim_paths_expr="/World/envs/env.*/taser")

        self.world.scene.add(robot)
        self.world.scene.add_default_ground_plane(z_position=-0.65)

        return robot

    def on_physics_step(self, step_size: float) -> None:
        if self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False

            self.step: int = 0
            self.max_step: int = 0
            self.episode: CuroboEpisode = None
            self.dataset_episodes: list[GPTEpisode] = []
        else:
            actions = np.zeros((self.num_envs, self.num_total_dof))
            actions[:, self.dof_ids.locks] = [-0.5, 7.0, -0.5, 7.0]

            if self.episode is None or self.step >= self.max_step:
                self.robot.apply_action(ArticulationActions(joint_velocities=actions))
                return

            current_joint_pos = self.robot.get_joint_positions().copy()
            current_joint_vel = self.robot.get_joint_velocities().copy()

            # Apply the joint velocity at the current step of the plan or zero if the plan has finished
            dq_ref = self.episode.joint_velocities.reshape(
                self.num_envs, -1, self.num_arm_dof
            )[:, self.step]
            q_ref = self.episode.joint_positions.reshape(
                self.num_envs, -1, self.num_arm_dof
            )[:, self.step]

            # Add extra PD control for better tracking
            pos_err = q_ref - current_joint_pos[:, self.arm_dof_ids]
            vel_err = dq_ref - current_joint_vel[:, self.arm_dof_ids]

            actions[:, self.arm_dof_ids] = dq_ref + Kp * (pos_err) + Kd * (vel_err)

            for env_idx in range(self.num_envs):
                both_arms_success = np.all(self.episode.episode_length[env_idx] > 0)
                is_done = self.step >= np.max(self.episode.episode_length[env_idx])
                if not both_arms_success or is_done:
                    continue

                is_last_step = self.step == self.max_step - 1
                if is_last_step:
                    # Hold the last position
                    actions[env_idx, self.arm_dof_ids] = 0.0

                self.dataset_episodes[env_idx].add_observation(
                    GPTEpisode.collect_observation(
                        joint_positions=current_joint_pos[env_idx, self.arm_dof_ids],
                        joint_velocities=current_joint_vel[env_idx, self.arm_dof_ids],
                        eef_positions=self.episode.eef_positions[
                            env_idx, self.step
                        ].flatten(),
                        target_positions=self.episode.target_position[
                            env_idx
                        ].flatten(),
                    )
                )
                self.dataset_episodes[env_idx].add_action(
                    actions[env_idx, self.arm_dof_ids],
                )

            self.robot.apply_action(ArticulationActions(joint_velocities=actions))
            self.step += 1

    def run(self) -> None:
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_stopped():
                self.needs_reset = True

            if self.step < self.max_step:
                continue

            for gpt_episode in self.dataset_episodes:
                if gpt_episode.observations.shape[0] == 0:
                    continue  # Skip unsuccessful episodes
                gpt_episode.save_to_h5(
                    file_path=self.output_path,
                    episode_id=str(self.num_plans_saved),
                )
                self.num_plans_saved += 1
                self.pbar.update(1)
            self.dataset_episodes = []

            if self.num_plans_saved >= self.num_plans_to_save:
                break

            self.pbar.set_postfix_str(
                f"Generating plans for {self.num_envs} envs | Total Saved: {self.num_plans_saved}",
            )

            self.episode = self.planner.plan()
            if self.episode is None:
                continue

            # Store start configurations in order to execute them in parallel
            start_cfgs = np.zeros((self.num_envs, self.num_total_dof))
            start_cfgs[:, self.dof_ids.left_arm] = self.episode.start_cfg[:, 0, :]
            start_cfgs[:, self.dof_ids.right_arm] = self.episode.start_cfg[:, 1, :]
            for env_idx in range(self.num_envs):
                VisualCuboid(
                    prim_path=f"/World/targets/target_left{env_idx}",
                    position=self.episode.target_position[env_idx, 0]
                    + self.env_origins[env_idx],
                    scale=np.array([0.1, 0.1, 0.1]),
                    color=np.array([0.63, 0.0, 0.8]),
                )
                VisualCuboid(
                    prim_path=f"/World/targets/target_right{env_idx}",
                    position=self.episode.target_position[env_idx, 1]
                    + self.env_origins[env_idx],
                    scale=np.array([0.1, 0.1, 0.1]),
                    color=np.array([0.0, 0.63, 0.8]),
                )

            self.robot.set_world_poses(
                self.env_origins,
                np.array([1.0, 0.0, 0.0, 0.0])[None, :].repeat(self.num_envs, axis=0),
            )

            # Initialize the robot envs at the start configurations of their current plans
            self.robot.set_joint_positions(start_cfgs)
            self.robot.set_joint_velocities(
                np.zeros((self.num_envs, self.num_total_dof))
            )

            # Prepare to store the executed plans in dataset format
            self.dataset_episodes = [GPTEpisode() for _ in range(self.num_envs)]

            self.step = 0
            self.max_step = self.episode.joint_velocities.shape[1]


def main():
    sim = CuroboDatasetCollector()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.setup()
    simulation_app.update()
    sim.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
