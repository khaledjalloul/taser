import math

import numpy as np
import torch

from isaaclab.envs import mdp
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.utils import configclass
from taser.isaaclab.common.base_env_cfg import BaseTaserEnvCfg
from taser.isaaclab.common.obs_utils import base_quat_w, base_vel_w


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
        ),
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
                    "base_left_arm_shoulder_joint",
                    "left_arm_1_left_arm_2_joint",
                    "left_arm_2_left_arm_3_joint",
                    "base_right_arm_shoulder_joint",
                    "right_arm_1_right_arm_2_joint",
                    "right_arm_2_right_arm_3_joint",
                    "base_left_wheel_joint",
                    "base_right_wheel_joint",
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


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Current planar velocity
        base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
        base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)

        # Target planar velocity
        target_vel_b = ObservationTermCfg(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}
        )

    @configclass
    class ProprioCfg(ObservationGroupCfg):
        """Proprioceptive observations."""

        # Base orientation useful for balancing
        base_quat_w = ObservationTermCfg(func=base_quat_w)

        # Base velocity useful for balancing
        base_vel_w = ObservationTermCfg(func=base_vel_w)

        # Joint states
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    proprio: ProprioCfg = ProprioCfg()


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

    track_lin_vel_xy_exp = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )


# def lin_vel_error_too_large(env: ManagerBasedEnv):
#     return (
#         torch.abs(env.target_vel_b[:, 0] - root_planar_vel_b(env)[:, 0])
#         > env._max_lin_vel * 1.2
#     )


# def ang_vel_error_too_large(env: ManagerBasedEnv):
#     return (
#         torch.abs(env.target_vel_b[:, 1] - root_planar_vel_b(env)[:, 1])
#         > env._max_ang_vel * 1.2
#     )


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

    # lin_vel_error_too_large = TerminationTermCfg(
    #     func=lin_vel_error_too_large,
    # )

    # ang_vel_error_too_large = TerminationTermCfg(
    #     func=ang_vel_error_too_large,
    # )


@configclass
class TaserEnvCfg(BaseTaserEnvCfg):
    """TASER environment configuration for the track velocity task."""

    commands = CommandsCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
