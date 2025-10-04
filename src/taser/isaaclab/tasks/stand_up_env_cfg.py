import math

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

from taser.isaaclab.common.articulation import TASER_CONFIG_USD
from taser.isaaclab.common.base_env_cfg import TaserBaseEnvCfg, TaserBaseSceneCfg


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    arm_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_link_left_arm_shoulder_joint",
            "left_arm_1_left_arm_2_joint",
            "left_arm_2_left_arm_3_joint",
            "base_link_right_arm_shoulder_joint",
            "right_arm_1_right_arm_2_joint",
            "right_arm_2_right_arm_3_joint",
        ],
        scale=5.0,
    )

    wheel_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_link_left_wheel_joint",
            "base_link_right_wheel_joint",
        ],
        scale=3.0,
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
            "position_range": (-0.3, 0.3),
            "velocity_range": (-0.6, 0.6),
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
                # Randomized starting pitch to start fallen down
                "pitch": (-math.pi / 2, math.pi / 2),
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

        # Base link velocity in base frame
        base_lin_vel_b = ObservationTermCfg(func=mdp.base_lin_vel)
        base_ang_vel_b = ObservationTermCfg(func=mdp.base_ang_vel)

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Base orientation useful for balancing
        # NOTE: Should be in proprio group but keeping here to get a policy observation
        base_quat_w = ObservationTermCfg(func=mdp.root_quat_w)

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


def zero_linear_velocity_reward(env: ManagerBasedEnv, std: float = 1.0):
    """Get the reward based on the robot's linear velocity in world frame."""
    root_lin_vel_b = mdp.base_lin_vel(env)
    return torch.exp(
        -(torch.linalg.vector_norm(root_lin_vel_b, dim=-1) ** 2) / (2 * std**2)
    )


def zero_roll_and_yaw_velocity_reward(env: ManagerBasedEnv, std: float = 1.0):
    """Get the reward based on the robot's angular velocity in base frame."""
    root_ang_vel_b = mdp.base_ang_vel(env)
    roll_yaw_ang_vel = torch.cat((root_ang_vel_b[:, 0], root_ang_vel_b[:, 2]), dim=-1)
    return torch.exp(
        -(torch.linalg.vector_norm(roll_yaw_ang_vel, dim=-1) ** 2) / (2 * std**2)
    )


def straighten_up_reward(env: ManagerBasedEnv):
    """Get the reward based on how straight the robot is."""
    base_quat = mdp.root_quat_w(env)
    # Reward is 1 when base_quat is (1, 0, 0, 0) and decreases as it deviates
    return base_quat[:, 0]  # Assuming base_quat is normalized


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    alive_reward = RewardTermCfg(func=mdp.is_alive, weight=1.0)

    tilt_penalty = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-10.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    zero_linear_velocity_reward = RewardTermCfg(
        func=zero_linear_velocity_reward, weight=3.0
    )

    zero_roll_and_yaw_velocity_reward = RewardTermCfg(
        func=zero_roll_and_yaw_velocity_reward, weight=3.0
    )

    # joint_vel_penalty = RewardTermCfg(
    #     func=mdp.joint_vel_l2,
    #     weight=-0.2,
    #     params={"asset_cfg": SceneEntityCfg("robot")},
    # )


@configclass
class SceneCfg(TaserBaseSceneCfg):
    """Scene for the stand up task."""

    robot = TASER_CONFIG_USD.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)


@configclass
class TaserStandUpEnvCfg(TaserBaseEnvCfg):
    """TASER environment configuration for the stand up task."""

    actions = ActionsCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    scene = SceneCfg()
    terminations = TerminationsCfg()
