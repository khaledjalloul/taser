import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    left_arm_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[
            "base_left_arm_1_joint",
            "left_arm_1_left_arm_2_joint",
            "left_arm_2_left_arm_3_joint"
        ],
        scale=10.0
    )

    right_arm_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[
            "base_right_arm_1_joint",
            "right_arm_1_right_arm_2_joint",
            "right_arm_2_right_arm_3_joint"
        ],
        scale=10.0
    )

    wheel_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[
            "base_wheel_1_joint",
            "base_wheel_2_joint"
        ],
        scale=10000.0
    )
