import math

import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass
from isaaclab.managers import SceneEntityCfg, EventTermCfg


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "base_left_arm_1_joint",
                    "left_arm_1_left_arm_2_joint",
                    "left_arm_2_left_arm_3_joint",
                    "base_right_arm_1_joint",
                    "right_arm_1_right_arm_2_joint",
                    "right_arm_2_right_arm_3_joint",
                    "base_wheel_1_joint",
                    "base_wheel_2_joint"
                ]
            ),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )
