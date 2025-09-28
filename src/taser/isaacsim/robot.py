from pathlib import Path

import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationActions
from omni.isaac.core.prims import XFormPrim

from taser.isaaclab.rl.custom import ActorCritic
from taser.isaacsim.utils.occupancy_grid import OccupancyGrid
from taser.isaacsim.utils.ros2_tf_publisher import add_tf_publisher
from taser.manipulation import IKManipulator

NAME = "taser"
PRIM_PATH = "/World/Taser"
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
USD_PATH = str(
    Path(get_package_share_directory("taser_ros"))
    / "robot_description"
    / "usd"
    / "taser.usd"
)
POLICY_PATH = Path("/workspaces/taser/src/taser/isaaclab/trained_models/balance.pth")


class TaserIsaacSimRobot(XFormPrim):
    def __init__(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float],
    ) -> None:
        """
        Initialize the robot.

        Args:
            position (tuple[float, float, float]): Initial position of the robot in meters.
            orientation (tuple[float, float, float]): Initial orientation of the robot as a quaternion [w, x, y, z].
        """
        self._position = np.array(position)
        self._orientation = np.array(orientation)

        self._prim = add_reference_to_stage(usd_path=USD_PATH, prim_path=PRIM_PATH)

        self._load_policy_model()

        super().__init__(
            name=NAME,
            prim_path=PRIM_PATH,
            position=position,
            orientation=orientation,
        )

        add_tf_publisher(
            robot_name=NAME,
            target_prim=f"{PRIM_PATH}/base_wrapper",
            tf_publisher_topic="/tf",
        )

        self._articulation = Articulation(prim_paths_expr=PRIM_PATH, name=NAME)

        self._manipulator = IKManipulator()

    def step(self, dt: float, occupancy_grid: OccupancyGrid) -> None:
        obs_dict = self._get_observations()
        joint_velocities = self._get_action(obs_dict)
        action = ArticulationActions(
            joint_velocities=joint_velocities,
            # joint_names=self._articulation.dof_names,
        )
        self._articulation.apply_action(action)

    def _load_policy_model(self) -> None:
        obs_dim = 29
        act_dim = 8
        self._model = ActorCritic(obs_dim=obs_dim, act_dim=act_dim).to(DEVICE)

        self._model.load(POLICY_PATH)
        self._model.eval()

    def _get_observations(self) -> torch.Tensor:
        base_link = f"{PRIM_PATH}/base"
        base_link_idx = self._articulation._physics_view.link_paths[0].index(base_link)

        joint_pos = self._articulation.get_joint_positions()
        joint_vel = self._articulation.get_joint_velocities()

        base_vel = self._articulation._physics_view.get_link_velocities()[
            :, base_link_idx
        ]
        base_pose = self._articulation._physics_view.get_link_transforms()[
            :, base_link_idx
        ]
        base_pos = base_pose[:, :3]
        base_quat = np.roll(base_pose[:, 3:], 1, axis=-1)  # w, x, y, z

        proprio_obs = np.concatenate(
            (
                joint_pos,
                joint_vel,
                base_quat,
                base_vel,
            ),
            axis=-1,
        )

        policy_obs = np.concatenate(
            (base_pos,),
            axis=-1,
        )

        obs_dict = {
            "proprio": torch.tensor(proprio_obs, dtype=torch.float32, device=DEVICE),
            "policy": torch.tensor(policy_obs, dtype=torch.float32, device=DEVICE),
        }
        return obs_dict

    def _get_action(self, obs_dict: dict[str, torch.Tensor]) -> np.ndarray:
        action_dist, _ = self._model(obs_dict)
        action = action_dist.mean.detach().cpu().numpy().flatten()

        action[0] = action[0] * 10.0  # left arm shoulder joint
        action[1] = action[1] * 100.0  # left wheel joint
        action[2] = action[2] * 10.0  # right arm shoulder joint
        action[3] = action[3] * 100.0  # right wheel joint
        action[4] = action[4] * 10.0  # left arm joint 2
        action[5] = action[5] * 10.0  # right arm joint 2
        action[6] = action[6] * 10.0  # left arm joint 3
        action[7] = action[7] * 10.0  # right arm joint 3

        return action
