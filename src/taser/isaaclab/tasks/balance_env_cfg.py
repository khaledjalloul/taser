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

from taser.isaaclab.common.articulation import TASER_CONFIG_USD
from taser.isaaclab.common.base_env_cfg import TaserBaseEnvCfg, TaserBaseSceneCfg


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    wheel_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_link_left_wheel_joint",
            "base_link_right_wheel_joint",
        ],
        scale=5.0,
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
            "position_range": (-torch.pi / 2, torch.pi / 2),
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
                # Randomized starting orientation to help explore scenarios where the robot is about to fall
                "pitch": (-0.3, 0.3),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
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


def zero_velocity_reward(env: ManagerBasedEnv, std: float) -> torch.Tensor:
    """Get the reward based on the robot's velocity."""
    base_vel = torch.cat([mdp.base_lin_vel(env), mdp.base_ang_vel(env)], dim=-1)
    return torch.exp(-(torch.linalg.vector_norm(base_vel, dim=-1) ** 2) / std**2)


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

    zero_vel_reward = RewardTermCfg(
        func=zero_velocity_reward,
        weight=5.0,
        params={"std": 0.25},
    )


@configclass
class SceneCfg(TaserBaseSceneCfg):
    """Scene for the balance task."""

    robot = TASER_CONFIG_USD.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)

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
    scene = SceneCfg()
    terminations = TerminationsCfg()
