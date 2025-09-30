import math

import numpy as np
import torch
from isaaclab.envs import ManagerBasedEnv, mdp
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.utils import configclass

from taser.isaaclab.common.base_env_cfg import TaserBaseEnvCfg
from taser.isaaclab.common.obs_utils import base_pos_b, base_quat_w, base_vel_w


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    wheel_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_link_left_wheel_joint",
            "base_link_right_wheel_joint",
        ],
        scale=10.0,
    )


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
                    "base_link_left_arm_shoulder_joint",
                    "left_arm_1_left_arm_2_joint",
                    "left_arm_2_left_arm_3_joint",
                    "base_link_right_arm_shoulder_joint",
                    "right_arm_1_right_arm_2_joint",
                    "right_arm_2_right_arm_3_joint",
                    "base_link_left_wheel_joint",
                    "base_link_right_wheel_joint",
                ],
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
                # Randomized starting pitch to help explore scenarios where the robot is about to fall
                "pitch": (-0.3, 0.3),
                # "pitch": (0.0, 0.0),
                "yaw": (-math.pi, math.pi),
                # "yaw": (0.0, 0.0),
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


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class ProprioCfg(ObservationGroupCfg):
        """Proprioceptive observations."""

        # Joint states
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)

        # Base orientation useful for balancing
        base_quat_w = ObservationTermCfg(func=base_quat_w)

        # Base velocity useful for balancing
        # NOTE: Using world frame instead of body frame since the former is more stable
        # and leads to better performance
        base_vel_w = ObservationTermCfg(func=base_vel_w)

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Base position in environment frame to stay close to the origin
        base_pos_b = ObservationTermCfg(func=base_pos_b)

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


def origin_position_reward(env: ManagerBasedEnv, std: float = 1.0):
    """Get the distances from the robot's position to the environment's origin."""
    return torch.exp(
        -(torch.linalg.vector_norm(base_pos_b(env)[:, :2], dim=-1) ** 2) / (2 * std**2)
    )


def zero_velocity_reward(env: ManagerBasedEnv, std: float = 1.0):
    """Get the penalty based on the robot's velocity."""
    return torch.exp(
        -(torch.linalg.vector_norm(base_vel_w(env), dim=-1) ** 2) / (2 * std**2)
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    alive_reward = RewardTermCfg(func=mdp.is_alive, weight=1.0)

    termination_penalty = RewardTermCfg(func=mdp.is_terminated, weight=-10.0)

    tilt_penalty = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    origin_pos_reward = RewardTermCfg(func=origin_position_reward, weight=3.0)
    zero_vel_reward = RewardTermCfg(func=zero_velocity_reward, weight=3.0)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    robot_falling = TerminationTermCfg(
        func=mdp.bad_orientation,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "limit_angle": float(np.deg2rad(70.0)),
        },
    )


@configclass
class TaserBalanceEnvCfg(TaserBaseEnvCfg):
    """TASER environment configuration for the balance task."""

    actions = ActionsCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
