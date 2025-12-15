import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate"
)
parser.add_argument("--path", type=str, help="Input file path")
parser.add_argument(
    "--output_format",
    type=str,
    default="text",
    help="Output format",
    choices=["sim", "text"],
)

args = parser.parse_args()

###############################################################

from pathlib import Path

import numpy as np

from taser.common.logger import logger
from taser_training.BC.utils.dataset import GPTDataset, GPTEpisode

if not args.path:
    outputs_dir = Path("/workspace/taser") / "outputs" / "BC" / "datasets"
    files = sorted(outputs_dir.glob("*"))
    if not files:
        raise FileNotFoundError(f"No dataset found in {outputs_dir}.")
    args.path = files[-1]
    logger.info(f"No path provided. Using the latest dataset at {args.path}")


def sim():
    from isaacsim.simulation_app import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    import isaacsim.core.utils.stage as stage_utils
    import torch
    from isaacsim.core.api.objects import VisualCuboid
    from isaacsim.core.cloner import GridCloner
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from omni.isaac.core import World
    from omni.isaac.core.utils.types import ArticulationActions
    from pxr import UsdGeom  # type: ignore

    from taser.common.datatypes import TaserJointState
    from taser.common.model import USD_PATH

    class GPTEvaluator:
        def __init__(self):
            self.num_envs: int = args.num_envs
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

            self.world: World = World()
            self.robot = self._spawn_robot()
            self.env_origins = self.robot.get_world_poses()[0]
            self.world.initialize_physics()

            self.dataset = GPTDataset(file_path=args.path, action_chunk_size=1)

            self.dof_ids = TaserJointState.isaac_indices
            self.arm_dof_ids = np.hstack(
                [self.dof_ids.left_arm, self.dof_ids.right_arm]
            )
            self.num_total_dof = self.robot.num_dof

            self.needs_reset = True

        def setup(self) -> None:
            self.world.add_physics_callback(
                "taser_step", callback_fn=self.on_physics_step
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

            robot = Articulation(
                name="taser", prim_paths_expr="/World/envs/env.*/taser"
            )

            self.world.scene.add(robot)
            self.world.scene.add_default_ground_plane(z_position=-0.65)

            return robot

        def on_physics_step(self, step_size: float) -> None:
            if self.needs_reset:
                self.world.reset(True)
                self.needs_reset = False

                self.step = 0
                self.max_step = 0
                self.dataset_idx = 0
                self.episodes = [None for _ in range(self.num_envs)]
            else:
                actions = np.zeros((self.num_envs, self.num_total_dof))
                actions[:, self.dof_ids.locks] = [-0.5, 7.0, -0.5, 7.0]

                for env_idx in range(self.num_envs):
                    if self.episodes[env_idx] is None:
                        continue

                    if self.step < self.episodes[env_idx].shape[0]:
                        actions[env_idx, self.arm_dof_ids] = self.episodes[env_idx][
                            self.step
                        ]
                    else:
                        actions[env_idx, self.arm_dof_ids] = self.episodes[env_idx][-1]

                self.robot.apply_action(ArticulationActions(joint_velocities=actions))
                self.step += 1

        def run(self) -> None:
            while simulation_app.is_running():
                self.world.step(render=True)
                if self.world.is_stopped():
                    self.needs_reset = True

                if self.step < self.max_step:
                    continue

                # Store start configurations in order to execute them in parallel
                start_cfgs = np.zeros((self.num_envs, self.num_total_dof))

                for env_idx in range(self.num_envs):
                    ep = self.dataset.get_full_episode(self.dataset_idx)
                    obs = ep["observations"]
                    self.episodes[env_idx] = ep["actions"]

                    start_cfgs[env_idx, self.arm_dof_ids] = obs[
                        0, GPTEpisode.indices["joint_positions"]
                    ]

                    VisualCuboid(
                        prim_path=f"/World/targets/target_left{env_idx}",
                        position=obs[0, GPTEpisode.indices["target_positions"]][:3]
                        + self.env_origins[env_idx],
                        scale=np.array([0.1, 0.1, 0.1]),
                        color=np.array([0.63, 0.0, 0.8]),
                    )
                    VisualCuboid(
                        prim_path=f"/World/targets/target_right{env_idx}",
                        position=obs[0, GPTEpisode.indices["target_positions"]][3:6]
                        + self.env_origins[env_idx],
                        scale=np.array([0.1, 0.1, 0.1]),
                        color=np.array([0.0, 0.63, 0.8]),
                    )
                    self.dataset_idx += 1

                self.robot.set_world_poses(
                    self.env_origins,
                    np.array([1.0, 0.0, 0.0, 0.0])[None, :].repeat(
                        self.num_envs, axis=0
                    ),
                )

                # Initialize the robot envs at the start configurations of their current plans
                self.robot.set_joint_positions(start_cfgs)
                self.robot.set_joint_velocities(
                    np.zeros((self.num_envs, self.num_total_dof))
                )

                self.step = 0
                self.max_step = max(
                    [ep.shape[0] for ep in self.episodes if ep is not None]
                )

    sim = GPTEvaluator()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.setup()
    simulation_app.update()
    sim.run()
    simulation_app.close()


def print_text_summary():
    import os
    import time

    dataset = GPTDataset(file_path=args.path, action_chunk_size=1)
    total_episodes = len(dataset)
    ep_lengths = np.array([])

    for ep_id in range(total_episodes):
        os.system("clear")

        episode = dataset.get_full_episode(ep_id)
        obs = episode["observations"]
        actions = episode["actions"]

        ep_lengths = np.append(ep_lengths, obs.shape[0])

        logger.info(f"Episode: {ep_id + 1} / {total_episodes}")
        logger.info(f"Observations shape: {obs.shape}")
        logger.info(f"Actions shape: {actions.shape}")
        logger.info(f"Episode length (episode {ep_id + 1}): {len(obs)}")
        logger.info(f"Mean episode length: {np.mean(ep_lengths)}")

        logger.info(f"First observation (episode 1): {obs[0]}")
        logger.info(f"First action (episode 1): {actions[0]}")

        logger.info(f"Last observation (episode {ep_id + 1}): {obs[-1]}")
        logger.info(f"Last action (episode {ep_id + 1}): {actions[-1]}")

        time.sleep(0.01)


def main():
    if args.output_format == "sim":
        sim()
    elif args.output_format == "text":
        print_text_summary()


if __name__ == "__main__":
    main()
