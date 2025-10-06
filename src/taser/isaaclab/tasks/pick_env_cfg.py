import math

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
import torch
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import ManagerBasedEnv, mdp
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


@configclass
class CurriculumCfg:
    """Curriculum specifications for the MDP."""

    enable_arms_near_target_reward_y = CurriculumTermCfg(
        func=mdp.modify_reward_weight,
        params={
            "term_name": "arms_near_target_reward_y",
            "weight": 10,
            "num_steps": 100_000,
        },
    )

    enable_target_picked_reward = CurriculumTermCfg(
        func=mdp.modify_reward_weight,
        params={
            "term_name": "target_picked_reward",
            "weight": 10,
            "num_steps": 200_000,
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


def target_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    return math_utils.transform_points(
        points=mdp.root_pos_w(env, asset_cfg=SceneEntityCfg("target")).unsqueeze(1),
        pos=-mdp.root_pos_w(env),
        quat=math_utils.quat_inv(mdp.root_quat_w(env)),
    )[:, 0, :]


def left_arm_eef_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    robot: RigidObject = env.scene["robot"]
    left_arm_eef_idx = robot.data.body_names.index("left_arm_eef")
    left_arm_eef_pos_w = robot.data.body_link_pos_w[:, left_arm_eef_idx]
    return math_utils.transform_points(
        points=left_arm_eef_pos_w.unsqueeze(1),  # (N, 1, 3)
        pos=-mdp.root_pos_w(env),
        quat=math_utils.quat_inv(mdp.root_quat_w(env)),
    )[:, 0, :]


def right_arm_eef_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    robot: RigidObject = env.scene["robot"]
    right_arm_eef_idx = robot.data.body_names.index("right_arm_eef")
    right_arm_eef_pos_w = robot.data.body_link_pos_w[:, right_arm_eef_idx]
    return math_utils.transform_points(
        points=right_arm_eef_pos_w.unsqueeze(1),  # (N, 1, 3)
        pos=-mdp.root_pos_w(env),
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
        left_arm_eef_pos_b = ObservationTermCfg(func=left_arm_eef_pos_b)
        right_arm_eef_pos_b = ObservationTermCfg(func=right_arm_eef_pos_b)

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


def arms_near_target_reward_in_axis(env: ManagerBasedEnv, axis: int, std: float = 0.25):
    """Reward for having both arms near the target in the specified axis."""
    left_arm_eef_pos_b_val = left_arm_eef_pos_b(env)
    right_arm_eef_pos_b_val = right_arm_eef_pos_b(env)
    target_pos_b_val = target_pos_b(env)

    reward_left = torch.exp(
        -torch.square(left_arm_eef_pos_b_val[:, axis] - target_pos_b_val[:, axis])
        / std**2
    )
    reward_right = torch.exp(
        -torch.square(right_arm_eef_pos_b_val[:, axis] - target_pos_b_val[:, axis])
        / std**2
    )
    return 0.6 * (reward_left + reward_right) / 2 + 0.4 * torch.minimum(
        reward_left, reward_right
    )


def target_picked_reward(
    env: ManagerBasedEnv, height_b: float = 0.4, std: float = 0.25
):
    """Reward for picking the object at a certain height."""
    return torch.exp(-torch.square(target_pos_b(env)[:, 2] - height_b) / std**2)


def target_near_robot_reward(env: ManagerBasedEnv, std: float = 0.25):
    """Reward for having the target near the robot base."""
    return torch.exp(-torch.square(target_pos_b(env)[:, 0]) / std**2)


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    arms_near_target_reward_x = RewardTermCfg(
        func=arms_near_target_reward_in_axis, params={"axis": 0}, weight=10.0
    )
    arms_near_target_reward_y = RewardTermCfg(
        func=arms_near_target_reward_in_axis,
        params={"axis": 1},
        weight=0.0,  # Enabled in curriculum
    )
    arms_near_target_reward_z = RewardTermCfg(
        func=arms_near_target_reward_in_axis, params={"axis": 2}, weight=10.0
    )

    target_picked_reward = RewardTermCfg(
        func=target_picked_reward,
        weight=0.0,  # Enabled in curriculum
    )

    target_near_robot_reward = RewardTermCfg(func=target_near_robot_reward, weight=5.0)


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
