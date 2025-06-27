import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.utils import configclass
from isaaclab.managers import SceneEntityCfg, EventTermCfg


def reset_target_velocity_t(env: ManagerBasedEnv, env_ids: torch.Tensor):
    # env._t[env_ids] = 0.0
    env._generate_target_vel(env_ids)


@configclass
class EventsCfg:
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

    reset_robot_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (-0.3, 0.3),
                # "pitch": (0.0, 0.0),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_target_velocity_t = EventTermCfg(
        func=reset_target_velocity_t,
        mode="reset",
    )
