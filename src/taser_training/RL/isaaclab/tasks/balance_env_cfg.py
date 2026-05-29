import numpy as np
from isaaclab.utils import configclass

from taser_training.RL.isaaclab.tasks.track_velocity_env_cfg import (
    JOINT_INDICES,
    TaserTrackVelocityEnvCfg,
)


@configclass
class TaserBalanceEnvCfg(TaserTrackVelocityEnvCfg):
    """TASER environment configuration for the balance task."""

    max_num_ppo_updates = 4_000

    curriculum = None

    def __post_init__(self):
        super().__post_init__()

        self.actions.wheel_velocities.scale = 5.0

        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.debug_vis = False

        self.events.reset_robot_lock_joints.position_range = (0.03, 0.15)
        self.events.reset_robot_lock_support_joints.position_range = (
            -np.deg2rad(90),
            np.deg2rad(10),
        )
        self.events.set_random_joint_velocities.params = {
            "joint_vels": {
                # [range, min]
                JOINT_INDICES.locks[0]: [2.0, -1.0],
                JOINT_INDICES.locks[1]: [20.0, -10.0],
                JOINT_INDICES.locks[2]: [2.0, -1.0],
                JOINT_INDICES.locks[3]: [20.0, -10.0],
            }
        }

        self.rewards.tilt_penalty.weight = -15.0
        self.rewards.track_lin_vel_xy.params["std"] = 0.1
        self.rewards.track_lin_vel_xy_global = None
        self.rewards.track_ang_vel_z.params["std"] = 0.1
        self.rewards.track_ang_vel_z_global = None
