import math

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
import torch
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnv, mdp
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
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from taser.isaaclab.common.articulation import TASER_CONFIG_FIXED_BASE_USD
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
        scale=10.0,
    )

    # left_arm_eef_vel = mdp.DifferentialInverseKinematicsActionCfg(
    #     asset_name="robot",
    #     body_name="left_arm_eef",
    #     controller=mdp.DifferentialIKControllerCfg(
    #         command_type="position", ik_method="pinv", ik_params={"k_val": 10}
    #     ),
    #     joint_names=[
    #         "base_link_left_arm_shoulder_joint",
    #         "left_arm_1_left_arm_2_joint",
    #         "left_arm_2_left_arm_3_joint",
    #     ],
    #     # scale=10.0,
    # )

    # right_arm_eef_vel = mdp.DifferentialInverseKinematicsActionCfg(
    #     asset_name="robot",
    #     body_name="right_arm_eef",
    #     controller=mdp.DifferentialIKControllerCfg(
    #         command_type="position", ik_method="pinv"
    #     ),
    #     joint_names=[
    #         "base_link_right_arm_shoulder_joint",
    #         "right_arm_1_right_arm_2_joint",
    #         "right_arm_2_right_arm_3_joint",
    #     ],
    #     scale=10.0,
    # )


def update_reward_weight(
    env: ManagerBasedRLEnv,
    env_ids,
    old_value,
    step: float,
    increment: float,
    max_value: float,
):
    """Update the reward weight."""
    weight = (env.common_step_counter // step) * increment
    return min(weight, max_value)


@configclass
class CurriculumCfg:
    """Curriculum specifications for the MDP."""

    enable_eefs_near_target_z_reward = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "rewards.eefs_near_target_z_reward.weight",
            "modify_fn": update_reward_weight,
            "modify_params": {"step": 5_000, "increment": 1, "max_value": 10},
        },
    )

    # enable_eefs_near_target_y_reward = CurriculumTermCfg(
    #     func=mdp.modify_term_cfg,
    #     params={
    #         "address": "rewards.eefs_near_target_y_reward.weight",
    #         "modify_fn": update_reward_weight,
    #         "modify_params": {"step": 10_000, "increment": 1, "max_value": 10},
    #     },
    # )

    enable_track_eef_velocity_y_reward = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "rewards.track_eef_velocity_y_reward.weight",
            "modify_fn": update_reward_weight,
            "modify_params": {"step": 10_000, "increment": 1, "max_value": 10},
        },
    )

    enable_target_picked_reward = CurriculumTermCfg(
        func=mdp.modify_term_cfg,
        params={
            "address": "rewards.target_picked_reward.weight",
            "modify_fn": update_reward_weight,
            "modify_params": {"step": 20_000, "increment": 1, "max_value": 10},
        },
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
                "pitch": (-0.1, 0.1),
                "yaw": (-0.2, 0.2),
                # "yaw": (0.0, 0.0),
            },
            "velocity_range": {},
        },
    )

    reset_pallet_position = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("pallet"),
            "pose_range": {
                "x": (0.65, 0.8),
                "y": (0.0, 0.0),
                "z": (0, 0),
            },
            "velocity_range": {},
        },
    )

    reset_target_position = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("target"),
            "pose_range": {
                "x": (0.4, 0.6),
                "y": (0.0, 0.0),
                "z": (0.7, 0.8),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-math.pi, math.pi),
            },
            "velocity_range": {},
        },
    )

    reset_target_scale = EventTermCfg(
        func=mdp.randomize_rigid_body_scale,
        mode="usd",
        params={
            "asset_cfg": SceneEntityCfg("target"),
            "scale_range": (0.5, 1.5),
        },
    )


def target_pos_b(env: ManagerBasedRLEnv) -> torch.Tensor:
    return math_utils.transform_points(
        points=env.scene["target"].data.root_pos_w.unsqueeze(1),  # (N, 1, 3)
        pos=-env.scene["robot"].data.root_pos_w,
        quat=math_utils.quat_inv(mdp.root_quat_w(env)),
    )[:, 0, :]


def arm_eef_state_b(env: ManagerBasedRLEnv, side: str, state: str) -> torch.Tensor:
    robot: RigidObject = env.scene["robot"]
    arm_eef_idx = robot.data.body_names.index(f"{side}_arm_eef")
    if state == "pos":
        arm_eef_state_w = robot.data.body_link_pos_w[:, arm_eef_idx]
    elif state == "vel":
        arm_eef_state_w = robot.data.body_link_lin_vel_w[:, arm_eef_idx]
    return math_utils.transform_points(
        points=arm_eef_state_w.unsqueeze(1),  # (N, 1, 3)
        pos=-robot.data.root_pos_w,
        quat=math_utils.quat_inv(mdp.root_quat_w(env)),
    )[:, 0, :]


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class ProprioCfg(ObservationGroupCfg):
        """Proprioceptive observations."""

        # Joint states
        joint_pos = ObservationTermCfg(func=mdp.joint_pos)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel)

        # Base orientation
        base_quat_w = ObservationTermCfg(func=mdp.root_quat_w)

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        target_pos_b = ObservationTermCfg(func=target_pos_b)
        left_arm_eef_pos_b = ObservationTermCfg(
            func=arm_eef_state_b, params={"side": "left", "state": "pos"}
        )
        right_arm_eef_pos_b = ObservationTermCfg(
            func=arm_eef_state_b, params={"side": "right", "state": "pos"}
        )

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


def eefs_near_target_reward_in_axis(env: ManagerBasedRLEnv, axis: int, std: float):
    """Reward for having both eefs near the target in the specified axis."""
    left_arm_eef_pos_b = arm_eef_state_b(env, "left", "pos")
    right_arm_eef_pos_b = arm_eef_state_b(env, "right", "pos")
    target_pos_b_val = target_pos_b(env)

    reward_left = torch.exp(
        -torch.square(left_arm_eef_pos_b[:, axis] - target_pos_b_val[:, axis]) / std**2
    )
    reward_right = torch.exp(
        -torch.square(right_arm_eef_pos_b[:, axis] - target_pos_b_val[:, axis]) / std**2
    )
    return 0.6 * (reward_left + reward_right) / 2 + 0.4 * torch.minimum(
        reward_left, reward_right
    )


def track_eef_velocity_y_reward_in_axis(
    env: ManagerBasedRLEnv,
    velocity: float,
    std: float,
):
    """Reward for having both arms near the target in the y-axis."""
    left_arm_eef_vel_b = arm_eef_state_b(env, "left", "vel")
    right_arm_eef_vel_b = arm_eef_state_b(env, "right", "vel")

    reward_left = torch.exp(
        -torch.square(left_arm_eef_vel_b[:, 1] - (-velocity)) / std**2
    )
    reward_right = torch.exp(
        -torch.square(right_arm_eef_vel_b[:, 1] - velocity) / std**2
    )
    return 0.6 * (reward_left + reward_right) / 2 + 0.4 * torch.minimum(
        reward_left, reward_right
    )


def target_picked_reward(env: ManagerBasedRLEnv, height_b: float, std: float):
    """Reward for picking the object at a certain height."""
    return torch.exp(-torch.square(target_pos_b(env)[:, 2] - height_b) / std**2)


def target_near_robot_reward(env: ManagerBasedRLEnv, std: float):
    """Reward for having the target near the robot base."""
    return torch.exp(-torch.square(target_pos_b(env)[:, 0]) / std**2)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    eefs_near_target_x_reward = RewardTermCfg(
        func=eefs_near_target_reward_in_axis,
        params={"axis": 0, "std": 0.25},
        weight=10.0,
    )

    eefs_near_target_z_reward = RewardTermCfg(
        func=eefs_near_target_reward_in_axis,
        params={"axis": 2, "std": 0.25},
        weight=0.0,  # Enabled in curriculum
    )

    # eefs_near_target_y_reward = RewardTermCfg(
    #     func=eefs_near_target_reward_in_axis,
    #     params={"axis": 1, "std": 1},
    #     weight=0.0,  # Enabled in curriculum
    # )

    track_eef_velocity_y_reward = RewardTermCfg(
        func=track_eef_velocity_y_reward_in_axis,
        params={"velocity": 0.2, "std": 0.25},
        weight=0.0,  # Enabled in curriculum
    )

    target_picked_reward = RewardTermCfg(
        func=target_picked_reward,
        params={"height_b": 0.4, "std": 0.25},
        weight=0.0,  # Enabled in curriculum
    )

    target_near_robot_reward = RewardTermCfg(
        func=target_near_robot_reward,
        params={"std": 0.25},
        weight=5.0,
    )


@configclass
class SceneCfg(TaserBaseSceneCfg):
    """Scene for the pick task."""

    robot = TASER_CONFIG_FIXED_BASE_USD.replace(prim_path="{ENV_REGEX_NS}/Robot")

    target = RigidObjectCfg(
        prim_path="/World/envs/env_.*/target",
        spawn=sim_utils.CuboidCfg(
            size=(0.4, 0.4, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0), metallic=0.2
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=4, solver_velocity_iteration_count=0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1000),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )

    pallet = RigidObjectCfg(
        prim_path="/World/envs/env_.*/pallet",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Pallet/o3dyn_pallet.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=10000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(rot=(0.707, 0, 0, 0.707)),
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)


@configclass
class TaserPickEnvCfg(TaserBaseEnvCfg):
    """TASER environment configuration for the pick task."""

    actions = ActionsCfg()
    curriculum = CurriculumCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    scene = SceneCfg()
    terminations = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        self.episode_length_s = 10
