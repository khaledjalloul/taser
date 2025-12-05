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

import numpy as np
import torch
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationActions
from tqdm import tqdm

from taser.common.datatypes import TaserJointState
from taser.common.model import USD_PATH
from taser_training.curobo.data_collection.utils.curobo_planner import (
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

        self.env_origins = np.zeros((self.num_envs, 3))

        self.world: World = World()
        robot_prim_path = "/World/taser"
        add_reference_to_stage(usd_path=str(USD_PATH), prim_path=robot_prim_path)
        self.robot = Articulation(
            name="taser",
            prim_paths_expr=robot_prim_path,
            positions=self.env_origins,
        )
        # TODO: Enable self-collisions
        self.robot.set_enabled_self_collisions(np.full((self.num_envs), False))
        self.world.scene.add(self.robot)
        self.world.scene.add_default_ground_plane(z_position=-0.65)
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

    def plan(self):
        plans = [None for _ in range(self.num_envs)]

        for env_idx in range(self.num_envs):
            self.pbar.set_postfix_str(
                f"Generating: {env_idx}/{self.num_envs} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
            )
            new_plans = {"left": None, "right": None}
            for side in ["left", "right"]:
                num_attempts = 0
                while new_plans[side] is None:
                    new_plans[side] = self.planner.plan(env_idx, side)
                    num_attempts += 1
                    if num_attempts > 5:
                        break

            if new_plans["left"] is not None and new_plans["right"] is not None:
                plans[env_idx] = new_plans

        return plans

    def execute(self, plans: list[dict[str, CuroboEpisode]]):
        num_total_dof = self.robot.num_dof
        dof_ids = TaserJointState.isaac_indices

        num_plans_generated = sum([len(env_plans) for env_plans in plans])
        num_plans_executed = 0
        self.pbar.set_postfix_str(
            f"Executing: 0 / {num_plans_generated} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
        )

        # Store start configurations in order to execute them in parallel
        start_cfgs = np.zeros((self.num_envs, num_total_dof))
        done_ids = np.zeros(self.num_envs, dtype=bool)

        # Load the first plan for each env if available
        for env_idx in range(self.num_envs):
            plan = plans[env_idx]
            if plan is None:
                # Mark the env executed if there are no more plans
                done_ids[env_idx] = True
                continue

            # Extract start configurations and target poses for the current plan of the current env
            start_cfgs[env_idx, dof_ids.left_arm] = plan["left"].start_cfg
            start_cfgs[env_idx, dof_ids.right_arm] = plan["right"].start_cfg

            # Spawn visualization cubes at the target poses
            VisualCuboid(
                prim_path=f"/World/targets/target{env_idx}_left",
                position=plan["left"].target_position,
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.63, 0.0, 0.8]),
            )
            VisualCuboid(
                prim_path=f"/World/targets/target{env_idx}_right",
                position=plan["right"].target_position,
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
        while not done_ids.all():
            # observations = collect_observations(
            #     env, target_positions, target_quaternions
            # )
            actions = np.zeros((self.num_envs, num_total_dof))
            actions[:, TaserJointState.isaac_indices.locks] = [-0.5, 7.0, -0.5, 7.0]

            current_joint_pos = self.robot.get_joint_positions()
            current_joint_vel = self.robot.get_joint_velocities()

            for env_idx in range(self.num_envs):
                if done_ids[env_idx]:
                    continue
                for side in ["left", "right"]:
                    arm_dof_ids = (
                        dof_ids.left_arm if side == "left" else dof_ids.right_arm
                    )
                    plan = plans[env_idx][side]

                    # Apply the joint velocity at the current step of the plan or zero if the plan has finished
                    dq_ref = (
                        plan.joint_velocities[step]
                        if step < len(plan.joint_velocities)
                        else 0.0
                    )
                    q_ref = (
                        plan.joint_positions[step]
                        if step < len(plan.joint_positions)
                        else plan.joint_positions[-1]
                    )

                    # Add extra PD control for better tracking
                    pos_err = q_ref - current_joint_pos[env_idx, arm_dof_ids]
                    vel_err = dq_ref - current_joint_vel[env_idx, arm_dof_ids]

                    actions[env_idx, arm_dof_ids] = (
                        dq_ref + Kp * (pos_err) + Kd * (vel_err)
                    )

                # Record data until the end of the plan plus one extra final step with zero actions
                if step <= max(
                    len(plan.joint_velocities) for plan in plans[env_idx].values()
                ):
                    # dataset_episodes[env_idx]["observations"].append(
                    #     observations[env_idx]
                    # )
                    dataset_episodes[env_idx]["actions"].append(actions[env_idx])
                elif not done_ids[env_idx]:
                    done_ids[env_idx] = True
                    # with lock_file, gzip.open(output_path, "ab") as f:
                    #     pickle.dump(
                    #         dataset_episodes[env_idx],
                    #         f,
                    #         protocol=pickle.HIGHEST_PROTOCOL,
                    #     )
                    self.num_plans_saved += 1
                    num_plans_executed += 1

                    self.pbar.update(self.num_plans_saved - self.pbar.n)
                    self.pbar.set_postfix_str(
                        f"Executing: {num_plans_executed} / {num_plans_generated} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
                    )

                    # Remove the executed plan from the list
                    plans[env_idx] = None

            self.robot.apply_action(ArticulationActions(joint_velocities=actions))
            self.world.step(render=True)
            step += 1

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self.first_step = False
        elif self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False
            self.first_step = True

    def run(self) -> None:
        while simulation_app.is_running():
            plans = self.plan()
            self.execute(plans=plans)

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
