import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[
            "base_left_arm_1_joint",
            "left_arm_1_left_arm_2_joint",
            "left_arm_2_left_arm_3_joint",
            "base_right_arm_1_joint",
            "right_arm_1_right_arm_2_joint",
            "right_arm_2_right_arm_3_joint",
            "base_wheel_1_joint",
            "base_wheel_2_joint"
        ],
        scale=5.0
    )
