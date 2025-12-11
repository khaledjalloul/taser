import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to simulate"
)
parser.add_argument("--model_path", type=str, help="Path to the trained model.")
parser.add_argument(
    "--export", type=str, help="Directory path to export the torch and ONNX models."
)
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

###############################################################

from pathlib import Path

import isaacsim.core.utils.stage as stage_utils
import numpy as np
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
from taser.manipulation.kinematics import ManipulationKinematics
from taser_training.BC.model.gpt import GPT
from taser_training.BC.utils.dataset import OBS_DIM, GPTEpisode


class GPTEvaluator:
    def __init__(self):
        self.num_envs: int = args.num_envs

        self.world: World = World()
        self.robot = self._spawn_robot()
        self.env_origins = self.robot.get_world_poses()[0]
        self.targets = self._spawn_targets()
        self.world.initialize_physics()

        self.kin_left = ManipulationKinematics(arm="left")
        self.kin_right = ManipulationKinematics(arm="right")

        if not args.model_path:
            outputs_dir = Path("/workspace/taser") / "outputs" / "BC" / "gpt_training"
            subdirs = sorted(
                [outputs_dir / d for d in outputs_dir.glob("*") if d.is_dir()]
            )
            args.model_path = subdirs[-1] / "best_model.pth"
            print(
                f"No model path provided. Using the latest model at {args.model_path}"
            )

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = GPT().to(self.device)
        self.model.load_state_dict(
            torch.load(args.model_path, map_location=self.device)
        )
        self.model.eval()

        self.dof_ids = TaserJointState.isaac_indices
        self.arm_dof_ids = np.hstack([self.dof_ids.left_arm, self.dof_ids.right_arm])
        self.num_total_dof = self.robot.num_dof

        self.needs_reset = True
        self.first_step = True

    def setup(self) -> None:
        self.world.add_physics_callback(
            "taser_gpt_step", callback_fn=self.on_physics_step
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

    def _spawn_targets(self):
        targets: list[dict[str, VisualCuboid]] = [
            {"left": None, "right": None} for _ in range(self.num_envs)
        ]
        for env_idx in range(self.num_envs):
            targets[env_idx]["left"] = VisualCuboid(
                name=f"target_left{env_idx}",
                prim_path=f"/World/targets/target_left{env_idx}",
                position=self.env_origins[env_idx] + [0.3, 0.25, 0.0],
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.63, 0.0, 0.8]),
            )
            self.world.scene.add(targets[env_idx]["left"])
            targets[env_idx]["right"] = VisualCuboid(
                name=f"target_right{env_idx}",
                prim_path=f"/World/targets/target_right{env_idx}",
                position=self.env_origins[env_idx] + [0.3, -0.25, 0.0],
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.0, 0.63, 0.8]),
            )
            self.world.scene.add(targets[env_idx]["right"])
        return targets

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self.first_step = False
        elif self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False
            self.first_step = True

    def run(self) -> None:
        while simulation_app.is_running():
            if self.world.is_stopped():
                self.needs_reset = True

            observations = np.zeros((self.num_envs, OBS_DIM))
            actions = np.zeros((self.num_envs, self.num_total_dof))
            actions[:, self.dof_ids.locks] = [-0.5, 7.0, -0.5, 7.0]

            current_joint_pos = self.robot.get_joint_positions().copy()
            current_joint_vel = self.robot.get_joint_velocities().copy()

            for env_idx in range(self.num_envs):
                target_positions = np.hstack(
                    [
                        self.targets[env_idx][side].get_world_pose()[0]
                        - self.env_origins[env_idx]
                        for side in ["left", "right"]
                    ]
                )

                taser_js = TaserJointState(
                    left_arm=current_joint_pos[env_idx, self.dof_ids.left_arm],
                    right_arm=current_joint_pos[env_idx, self.dof_ids.right_arm],
                )
                left_eef_pos = self.kin_left.get_eef_position(taser_js)
                right_eef_pos = self.kin_right.get_eef_position(taser_js)

                eef_positions = np.hstack(
                    [
                        left_eef_pos.x,
                        left_eef_pos.y,
                        left_eef_pos.z,
                        right_eef_pos.x,
                        right_eef_pos.y,
                        right_eef_pos.z,
                    ]
                )

                new_obs = GPTEpisode.collect_observation(
                    joint_positions=current_joint_pos[env_idx, self.arm_dof_ids],
                    joint_velocities=current_joint_vel[env_idx, self.arm_dof_ids],
                    eef_positions=eef_positions,
                    target_positions=target_positions,
                )
                observations[env_idx] = new_obs

            with torch.no_grad():
                model_outputs: dict[str, torch.Tensor] = self.model(
                    torch.tensor(observations, device=self.device).float()
                )

            actions[:, self.arm_dof_ids] = (
                model_outputs["actions"][:, 0, :].cpu().numpy()
            )

            self.robot.apply_action(ArticulationActions(joint_velocities=actions))
            self.world.step(render=True)


def main():
    sim = GPTEvaluator()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.setup()
    simulation_app.update()
    sim.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
