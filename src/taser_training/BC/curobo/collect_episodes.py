import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument(
    "--num_plans",
    type=int,
    default=float("inf"),
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

import isaacsim.core.utils.stage as stage_utils
import numpy as np
import torch
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.cloner import GridCloner
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationActions
from pxr import UsdGeom
from tqdm import tqdm

from taser.common.datatypes import TaserJointState
from taser.common.model import USD_PATH
from taser_training.BC.curobo.curobo_planner import (
    CuroboEpisode,
    CuroboPlanner,
)

Kp = 3.0
Kd = 0.2


class CuroboDatasetCollector:
    def __init__(self):
        self.num_envs: int = args.num_envs
        self.num_plans_to_save: int = args.num_plans
        self.num_plans_saved: int = 0
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.world: World = World()
        self.robot = self._spawn_robot()
        self.env_origins = self.robot.get_world_poses()[0]
        self.world.initialize_physics()

        self.planner = CuroboPlanner(
            scene=self.world.scene,
            robot=self.robot,
            interpolation_dt=self.world.get_physics_dt(),
            env_origins=torch.tensor(self.env_origins, device=self.device),
            is_static_terrain=True,
            device=self.device,
        )

        self.pbar = tqdm(total=self.num_plans_to_save, desc="Generating Plans")

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

    def plan(self) -> tuple[dict[str, CuroboEpisode | None], int]:
        self.pbar.set_postfix_str(
            f"Generating {self.num_envs} plans | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
        )

        plans = {"left": None, "right": None}
        for side in plans.keys():
            num_attempts = 0
            while num_attempts < 5 and plans[side] is None:
                plans[side] = self.planner.plan(side=side)
                num_attempts += 1
        max_step = (
            max(
                [
                    plans[side].joint_velocities.shape[1]
                    for side in plans.keys()
                    if plans[side] is not None
                ]
            )
            if any(plans[side] is not None for side in plans.keys())
            else 0
        )
        return plans, max_step

    def execute(self, plans: dict[str, CuroboEpisode | None], max_step: int) -> None:
        num_total_dof = self.robot.num_dof
        dof_ids = TaserJointState.isaac_indices

        num_plans_generated = sum([len(env_plans) for env_plans in plans])
        self.pbar.set_postfix_str(
            f"Executing: 0 / {num_plans_generated} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
        )

        # Store start configurations in order to execute them in parallel
        start_cfgs = np.zeros((self.num_envs, num_total_dof))
        if plans["left"] is not None:
            start_cfgs[:, dof_ids.left_arm] = plans["left"].start_cfg
        if plans["right"] is not None:
            start_cfgs[:, dof_ids.right_arm] = plans["right"].start_cfg

        for env_idx in range(self.num_envs):
            if plans["left"] is not None:
                VisualCuboid(
                    prim_path=f"/World/targets/target_left{env_idx}",
                    position=plans["left"].target_position[env_idx]
                    + self.env_origins[env_idx],
                    scale=np.array([0.1, 0.1, 0.1]),
                    color=np.array([0.63, 0.0, 0.8]),
                )
            if plans["right"] is not None:
                VisualCuboid(
                    prim_path=f"/World/targets/target_right{env_idx}",
                    position=plans["right"].target_position[env_idx]
                    + self.env_origins[env_idx],
                    scale=np.array([0.1, 0.1, 0.1]),
                    color=np.array([0.0, 0.63, 0.8]),
                )

        # Prepare to store the executed plans in dataset format
        dataset_episodes = [
            {
                "actions": [],
                "observations": [],
            }
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
        while step < max_step:
            # observations = collect_observations(
            #     env, target_positions, target_quaternions
            # )
            actions = np.zeros((self.num_envs, num_total_dof))
            actions[:, TaserJointState.isaac_indices.locks] = [-0.5, 7.0, -0.5, 7.0]

            current_joint_pos = self.robot.get_joint_positions()
            current_joint_vel = self.robot.get_joint_velocities()

            for side, plans_side in plans.items():
                if plans_side is None:
                    continue

                arm_dof_ids = dof_ids.left_arm if side == "left" else dof_ids.right_arm
                step_batch = np.where(
                    step < plans_side.joint_velocities.shape[1], step, -1
                )

                # Apply the joint velocity at the current step of the plan or zero if the plan has finished
                dq_ref = plans[side].joint_velocities[:, step_batch]
                q_ref = plans[side].joint_positions[:, step_batch]

                # Add extra PD control for better tracking
                pos_err = q_ref - current_joint_pos[:, arm_dof_ids]
                vel_err = dq_ref - current_joint_vel[:, arm_dof_ids]

                actions[:, arm_dof_ids] = dq_ref + Kp * (pos_err) + Kd * (vel_err)

            dataset_episodes[env_idx]["actions"].append(actions[env_idx])

            self.robot.apply_action(ArticulationActions(joint_velocities=actions))
            self.world.step(render=True)
            step += 1

        # with lock_file, gzip.open(output_path, "ab") as f:
        #     pickle.dump(
        #         dataset_episodes[env_idx],
        #         f,
        #         protocol=pickle.HIGHEST_PROTOCOL,
        #     )
        self.num_plans_saved += 1

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self.first_step = False
        elif self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False
            self.first_step = True

    def run(self) -> None:
        while simulation_app.is_running():
            plans, max_step = self.plan()
            self.execute(plans=plans, max_step=max_step)

            if self.num_plans_saved >= self.num_plans_to_save:
                break


def main():
    sim = CuroboDatasetCollector()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
