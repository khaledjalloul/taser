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
    "--num_plans_per_terrain",
    type=int,
    default=1,
    help="Number of IK plans to generate for each unique terrain",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate"
)
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

###############################################################

import torch
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
from taser_training.curobo.data_collection.utils.utils import (
    visualize_targets,
)


class CuroboDatasetCollector:
    def __init__(self):
        self.num_envs: int = args.num_envs
        self.num_plans_to_save: int = args.num_plans
        self.num_plans_per_terrain: int = args.num_plans_per_terrain
        self.num_plans_saved: int = 0
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.env_origins = torch.zeros((self.num_envs, 3), device=self.device)

        self.world: World = World()
        robot_prim_path = "/World/taser"
        add_reference_to_stage(usd_path=str(USD_PATH), prim_path=robot_prim_path)
        self.robot = Articulation(
            name="taser",
            prim_paths_expr=robot_prim_path,
            positions=self.env_origins.cpu(),
        )
        # TODO: Enable self-collisions
        self.robot.set_enabled_self_collisions(torch.full((self.num_envs, ), False))
        self.world.scene.add(self.robot)
        self.world.scene.add_default_ground_plane(z_position=-0.65)
        self.world.initialize_physics()

        self.planner = CuroboPlanner(
            scene=self.world.scene,
            robot=self.robot,
            interpolation_dt=self.world.get_physics_dt(),
            env_origins=self.env_origins,
            num_plans_per_terrain=self.num_plans_per_terrain,
            is_static_terrain=True,
            device=self.device,
        )

        self.pbar = tqdm(total=self.num_plans_to_save, desc="Generating Plans")

        self.needs_reset = False
        self.first_step = True
        self.world.add_physics_callback("taser_step", callback_fn=self.on_physics_step)

    def plan(self):
        plans = [[] for _ in range(self.num_envs)]
        self.pbar.set_postfix_str(
            f"Generating: 0 / {self.num_plans_per_terrain * self.num_envs} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
        )

        for env_idx in range(self.num_envs):
            # Generate a batch of plans for the current environment
            num_attempts = 0
            while len(plans[env_idx]) < self.num_plans_per_terrain:
                new_plans = self.planner.plan(env_idx)
                plans[env_idx].extend(
                    new_plans[: self.num_plans_per_terrain - len(plans[env_idx])]
                )

                self.pbar.set_postfix_str(
                    f"Generating: {sum([len(env_plans) for env_plans in plans])} / {self.num_plans_per_terrain * self.num_envs} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
                )

                num_attempts += 1
                if num_attempts > 5 and len(plans[env_idx]) == 0:
                    break

        return plans

    def execute(self, plans: list[list[CuroboEpisode]]):
        """
        Execute the given cuRobo plans in the simulator and save the executed episodes to a dataset file.

        Args:
            plans (list[list[CuroboEpisode]]): 2D list of generated plans for each environment
            env (MoleSimEnv): The simulation environment
            env_origins (torch.Tensor): The (x, y, z) origins of each environment
            output_path (Path): Path to the output dataset file
            lock_file (FileLock): File lock to ensure safe writing to the output file
        """
        num_total_dof = self.robot.num_dof
        dof_ids = TaserJointState.isaac_indices
        arm_dof_ids = dof_ids.left_arm.tolist() + dof_ids.right_arm.tolist()

        num_plans_generated = sum([len(env_plans) for env_plans in plans])
        num_plans_executed = 0
        self.pbar.set_postfix_str(
            f"Executing: 0 / {num_plans_generated} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
        )

        while sum([len(env_plans) for env_plans in plans]) > 0:
            # Store start configurations and target poses for all envs in tensors in order to execute them in parallel
            start_cfgs = torch.zeros((self.num_envs, num_total_dof), device=self.device)
            target_left_positions = torch.zeros((self.num_envs, 3), device=self.device)
            target_left_quaternions = torch.zeros(
                (self.num_envs, 4), device=self.device
            )
            target_right_positions = torch.zeros((self.num_envs, 3), device=self.device)
            target_right_quaternions = torch.zeros(
                (self.num_envs, 4), device=self.device
            )
            done_ids = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            # Load the first plan for each env if available
            for env_idx in range(self.num_envs):
                if not plans[env_idx]:
                    # Mark the env executed if there are no more plans
                    done_ids[env_idx] = True
                    continue
                plan = plans[env_idx][0]

                # Extract start configurations and target poses for the current plan of the current env
                start_cfgs[env_idx, arm_dof_ids] = torch.tensor(
                    plan.start_cfg, device=self.device
                )
                target_left_positions[env_idx] = torch.tensor(
                    plan.target_left_position, device=self.device
                )
                target_left_quaternions[env_idx] = torch.tensor(
                    plan.target_left_quaternion, device=self.device
                )
                target_right_positions[env_idx] = torch.tensor(
                    plan.target_right_position, device=self.device
                )
                target_right_quaternions[env_idx] = torch.tensor(
                    plan.target_right_quaternion, device=self.device
                )

                # Spawn visualization cubes at the target poses
                visualize_targets(
                    positions=torch.stack(
                        (
                            target_left_positions[env_idx],
                            target_right_positions[env_idx],
                        )
                    )
                    + self.env_origins[env_idx],
                    quaternions=torch.stack(
                        (
                            target_left_quaternions[env_idx],
                            target_right_quaternions[env_idx],
                        )
                    ),
                    prim_path=f"/World/targets/target{env_idx}",
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
                self.env_origins.cpu(),
                torch.tensor([1.0, 0.0, 0.0, 0.0])
                .unsqueeze(0)
                .repeat(self.num_envs, 1)
                .cpu(),
            )
            # Initialize the robot envs at the start configurations of their current plans
            self.robot.set_joint_positions(start_cfgs.cpu())
            self.robot.set_joint_velocities(torch.zeros((self.num_envs, num_total_dof)))
            self.world.step(render=True)

            # Execute the plans until all envs are done
            step = 0
            while not done_ids.all():
                # observations = collect_observations(
                #     env, target_positions, target_quaternions
                # )
                actions = torch.zeros(
                    (self.num_envs, num_total_dof), device=self.device
                )
                actions[:, TaserJointState.isaac_indices.locks] = torch.tensor(
                    [-0.5, 7.0, -0.5, 7.0], device=self.device
                )

                for env_idx in range(self.num_envs):
                    if done_ids[env_idx]:
                        continue

                    plan = plans[env_idx][0]

                    # Apply the joint velocity at the current step of the plan or zero if the plan has finished
                    dq_ref = (
                        torch.tensor(plan.joint_velocities[step], device=self.device)
                        if step < len(plan.joint_velocities)
                        else 0.0
                    )
                    q_ref = (
                        torch.tensor(plan.joint_positions[step], device=self.device)
                        if step < len(plan.joint_positions)
                        else torch.tensor(plan.joint_positions[-1], device=self.device)
                    )

                    # Add extra PD control for better tracking
                    Kp = 3.0
                    Kd = 0.2
                    pos_err = q_ref - torch.from_numpy(
                        self.robot.get_joint_positions(joint_indices=arm_dof_ids)[
                            env_idx
                        ]
                    ).to(self.device)
                    vel_err = dq_ref - torch.from_numpy(
                        self.robot.get_joint_velocities(joint_indices=arm_dof_ids)[
                            env_idx
                        ]
                    ).to(self.device)
                    actions[env_idx, arm_dof_ids] = (
                        dq_ref + Kp * (pos_err) + Kd * (vel_err)
                    )

                    # Record data until the end of the plan plus one extra final step with zero actions
                    if step <= len(plan.joint_velocities):
                        # dataset_episodes[env_idx]["observations"].append(
                        #     observations[env_idx].detach().cpu().tolist()
                        # )
                        dataset_episodes[env_idx]["actions"].append(
                            actions[env_idx].cpu().tolist()
                        )
                    elif not done_ids[env_idx]:
                        done_ids[env_idx] = True

                        if plan.lidar_point_cloud is not None:
                            dataset_episodes[env_idx]["lidar_observations"] = (
                                plan.lidar_point_cloud
                            )
                        if plan.height_map_point_cloud is not None:
                            dataset_episodes[env_idx]["height_map_observations"] = (
                                plan.height_map_point_cloud
                            )

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
                        plans[env_idx].pop(0)

                self.robot.apply_action(
                    ArticulationActions(joint_velocities=actions.cpu())
                )
                self.world.step(render=True)
                step += 1

            self.pbar.update(self.num_plans_saved - self.pbar.n)
            self.pbar.set_postfix_str(
                f"Executing: {num_plans_executed} / {num_plans_generated} | Total Saved: {self.num_plans_saved}{f' / {self.num_plans_to_save}' if self.num_plans_to_save != float('inf') else ''}",
            )

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
