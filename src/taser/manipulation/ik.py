from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from roboticstoolbox.robot import Robot


class IKManipulator:
    def __init__(self):
        taser_ros_share_dir = get_package_share_directory("taser_ros")
        urdf_path = (
            Path(taser_ros_share_dir) / "robot_description" / "urdf" / "taser.urdf"
        )

        self._right_arm = Robot.URDF(
            file_path=str(urdf_path),
            gripper="right_arm_eef",
        )

        self._left_arm = Robot.URDF(
            file_path=str(urdf_path),
            gripper="left_arm_eef",
        )

    def get_dq_from_linear_v(
        self,
        v_desired: np.ndarray,
        q_current: np.ndarray,
        arm: str,
    ) -> np.ndarray:
        if arm == "right":
            robot = self._right_arm
        else:
            robot = self._left_arm

        J = robot.jacob0(q_current, start="base")
        J_pos = J[0:3, :]
        dq = np.linalg.pinv(J_pos) @ v_desired
        return dq
