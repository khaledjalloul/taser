import numpy as np
import torch
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv, mdp
from isaaclab.managers import (
    CurriculumTermCfg,
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.utils import configclass

from taser.common.datatypes import TaserJointState
from taser_training.RL.isaaclab.articulation import (
    GRIPPER_JOINT_NAMES,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    TASER_CONFIG_USD,
    WHEEL_JOINT_NAMES,
)
from taser_training.RL.isaaclab.base_env_cfg import (
    TaserBaseEnvCfg,
    TaserBaseSceneCfg,
)

CMD_V_MAX = 3.0
CMD_W_MAX = 3.0

ACTION_PENALTY_WEIGHT = -0.05

# 50% unlocked & moving, 20% unlocked & standing, 20% locking & moving, 10% locking & standing
UNLOCKED_ENVS_SPLIT = 0.7
STANDING_ENVS_SPLIT = 0.3

CURRICULUM_CMD_PPO_END = 0.5  # Maximum percentage of PPO updates for curriculum to ramp up command difficulty
CURRICULUM_ACTION_PENALTY_PPO_START = 0.7  # PPO percentage to enable action penalty

JOINT_INDICES = TaserJointState.isaac_indices


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    wheel_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=WHEEL_JOINT_NAMES,
        # max wheel vel = max lin vel / wheel radius = 3m/s / 0.15rad = 20 rad/s
        # action scale = max wheel vel / model action space = 20 rad/s / 1.0 = 20.0
        scale=(CMD_V_MAX / 0.15) / 1.0,
    )


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(5.0, 5.0),
        debug_vis=True,
        rel_standing_envs=STANDING_ENVS_SPLIT,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-CMD_V_MAX, CMD_V_MAX),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-CMD_W_MAX, CMD_W_MAX),
        ),
    )


def update_target_velocity_command(
    env: ManagerBasedRLEnv, env_ids, old_value, max_value: float
):
    """Update the target velocity command."""
    max_ppo_step = env.unwrapped.cfg.max_num_ppo_updates * CURRICULUM_CMD_PPO_END
    range = (env.unwrapped.num_ppo_updates / max_ppo_step) * max_value
    range = min(range, max_value)
    return (-range, range)


def update_action_penalty_weight(env: ManagerBasedRLEnv, env_ids, old_value):
    """Update the action penalty."""
    min_ppo_step = (
        env.unwrapped.cfg.max_num_ppo_updates * CURRICULUM_ACTION_PENALTY_PPO_START
    )
    if env.unwrapped.num_ppo_updates > min_ppo_step:
        return ACTION_PENALTY_WEIGHT
    return 0.0


@configclass
class CurriculumCfg:
    """Curriculum specifications for the MDP."""

    update_target_lin_velocity = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.base_velocity.ranges.lin_vel_x",
            "modify_fn": update_target_velocity_command,
            "modify_params": {"max_value": CMD_V_MAX},
        },
    )

    update_target_ang_velocity = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "commands.base_velocity.ranges.ang_vel_z",
            "modify_fn": update_target_velocity_command,
            "modify_params": {"max_value": CMD_W_MAX},
        },
    )

    enable_action_penalty = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "rewards.action_penalty.weight",
            "modify_fn": update_action_penalty_weight,
        },
    )


def set_random_joint_velocities(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    unlocked_joint_vels: dict[int, list[float]],
    locking_joint_vels: dict[int, list[float]],
    unlocked_envs_split: float,
):
    robot: Articulation = env.scene["robot"]

    is_unlocked = torch.rand(len(env_ids), device=env.device) < unlocked_envs_split

    unlocked_vel_target = (
        torch.rand((len(env_ids), len(unlocked_joint_vels)), device=env.device)
        * torch.tensor(list(unlocked_joint_vels.values()), device=env.device)[:, 0]
        + torch.tensor(list(unlocked_joint_vels.values()), device=env.device)[:, 1]
    )
    locking_vel_target = (
        torch.rand((len(env_ids), len(locking_joint_vels)), device=env.device)
        * torch.tensor(list(locking_joint_vels.values()), device=env.device)[:, 0]
        + torch.tensor(list(locking_joint_vels.values()), device=env.device)[:, 1]
    )

    vel_target = torch.where(
        is_unlocked.unsqueeze(1),
        unlocked_vel_target,
        locking_vel_target,
    )

    robot.set_joint_velocity_target(
        vel_target,
        joint_ids=list(locking_joint_vels.keys()),
        env_ids=env_ids,
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
                    LEFT_ARM_JOINT_NAMES[0],
                    LEFT_ARM_JOINT_NAMES[1],
                    LEFT_ARM_JOINT_NAMES[3],
                    LEFT_ARM_JOINT_NAMES[5],
                    RIGHT_ARM_JOINT_NAMES[0],
                    RIGHT_ARM_JOINT_NAMES[1],
                    RIGHT_ARM_JOINT_NAMES[3],
                    RIGHT_ARM_JOINT_NAMES[5],
                ],
            ),
            "position_range": (-0.5, 0.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_remaining_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    LEFT_ARM_JOINT_NAMES[2],
                    LEFT_ARM_JOINT_NAMES[4],
                    RIGHT_ARM_JOINT_NAMES[2],
                    RIGHT_ARM_JOINT_NAMES[4],
                    *GRIPPER_JOINT_NAMES,
                ],
            ),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_robot_lock_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "base_link_front_lock_joint",
                    "base_link_back_lock_joint",
                ],
            ),
            "position_range": (0.03, 0.15),
            "velocity_range": (0.0, 0.0),
        },
    )

    reset_robot_lock_support_joints = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "front_lock_support_joint",
                    "back_lock_support_joint",
                ],
            ),
            "position_range": (-np.deg2rad(90), -np.deg2rad(10)),
            "velocity_range": (0.0, 0.0),
        },
    )

    set_random_joint_velocities = EventTermCfg(
        func=set_random_joint_velocities,
        mode="interval",
        interval_range_s=(0.0, 10.0),
        params={
            "unlocked_joint_vels": {
                # [range, min]
                JOINT_INDICES.locks[0]: [0.8, 0.2],
                JOINT_INDICES.locks[1]: [7.0, -10.0],
                JOINT_INDICES.locks[2]: [0.8, 0.2],
                JOINT_INDICES.locks[3]: [7.0, -10.0],
            },
            "locking_joint_vels": {
                # [range, min]
                JOINT_INDICES.locks[0]: [2.0, -1.0],
                JOINT_INDICES.locks[1]: [20.0, -10.0],
                JOINT_INDICES.locks[2]: [2.0, -1.0],
                JOINT_INDICES.locks[3]: [20.0, -10.0],
            },
            "unlocked_envs_split": UNLOCKED_ENVS_SPLIT,
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
                "pitch": (-0.45, 0.45),
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

        # Base orientation useful for balancing
        base_quat_w = ObservationTermCfg(func=mdp.root_quat_w)

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Target planar velocity
        target_vel_b = ObservationTermCfg(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}
        )

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    alive_reward = RewardTermCfg(func=mdp.is_alive, weight=0.5)

    termination_penalty = RewardTermCfg(func=mdp.is_terminated, weight=-30.0)

    tilt_penalty = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    track_lin_vel_xy = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )

    track_ang_vel_z = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )

    action_penalty = RewardTermCfg(func=mdp.action_l2, weight=ACTION_PENALTY_WEIGHT)


@configclass
class SceneCfg(TaserBaseSceneCfg):
    """Scene for the track velocity task."""

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
class TaserTrackVelocityEnvCfg(TaserBaseEnvCfg):
    """TASER environment configuration for the track velocity task."""

    max_num_ppo_updates = 6_000

    actions = ActionsCfg()
    commands = CommandsCfg()
    curriculum = CurriculumCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    scene = SceneCfg()
    terminations = TerminationsCfg()
