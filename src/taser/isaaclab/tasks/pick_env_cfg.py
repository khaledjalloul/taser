import math

import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import RigidObjectCfg
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
                "pitch": (0.0, 0.0),
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

    reset_target_position = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("target"),
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.2, 0.4),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-math.pi, math.pi),
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

    reset_target_scale = EventTermCfg(
        func=mdp.randomize_rigid_body_scale,
        mode="usd",
        params={
            "asset_cfg": SceneEntityCfg("target"),
            "scale_range": (0.5, 1.5),
        },
    )


def target_pos_b(env: ManagerBasedEnv) -> torch.Tensor:
    target_pos_w = mdp.root_pos_w(env, asset_cfg=SceneEntityCfg("target"))
    robot_pos_w = mdp.root_pos_w(env, asset_cfg=SceneEntityCfg("robot"))
    target_pos_b = target_pos_w - robot_pos_w
    return target_pos_b


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

        target_pos = ObservationTermCfg(func=target_pos_b)

    # Observation groups
    proprio: ProprioCfg = ProprioCfg()
    policy: PolicyCfg = PolicyCfg()


def object_picked_reward(env: ManagerBasedEnv, height_threshold: float = 0.6):
    """Reward for picking the object above a certain height."""
    target_pos_w = mdp.root_pos_w(env, asset_cfg=SceneEntityCfg("target"))
    reward = (target_pos_w[:, 2] > height_threshold).float()
    return reward


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    object_picked_reward = RewardTermCfg(func=object_picked_reward, weight=10.0)


@configclass
class SceneCfg(TaserBaseSceneCfg):
    """Scene for the pick task."""

    robot = TASER_CONFIG_FIXED_BASE_USD.replace(prim_path="{ENV_REGEX_NS}/Robot")

    target = RigidObjectCfg(
        prim_path="/World/envs/env_.*/target",
        # spawn=sim_utils.UsdFileCfg(
        #     usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/props/sm_whitecorrugatedbox_b/sm_whitecorrugatedbox_b20_brown_01.usd",
        #     # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/props/sm_whitecorrugatedbox_b/sm_whitecorrugatedbox_b22_brown_01.usd",
        #     # rigid_props=sim_utils.RigidBodyPropertiesCfg(
        #     #     rigid_body_enabled=True,
        #     # ),
        # ),
        spawn=sim_utils.CuboidCfg(
            size=(0.4, 0.4, 0.4),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0), metallic=0.2
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=4, solver_velocity_iteration_count=0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.2)),
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)


@configclass
class TaserPickEnvCfg(TaserBaseEnvCfg):
    """TASER environment configuration for the pick task."""

    actions = ActionsCfg()
    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    scene = SceneCfg()
    terminations = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        self.episode_length_s = 10
