import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from taser.isaaclab.common.articulation import TASER_CONFIG_USD


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    left_arm_efforts = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_left_arm_shoulder_joint",
            "left_arm_1_left_arm_2_joint",
            "left_arm_2_left_arm_3_joint",
        ],
        scale=500.0,
    )

    right_arm_efforts = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_right_arm_shoulder_joint",
            "right_arm_1_right_arm_2_joint",
            "right_arm_2_right_arm_3_joint",
        ],
        scale=500.0,
    )

    wheel_efforts = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "base_left_wheel_joint",
            "base_right_wheel_joint",
        ],
        scale=5000.0,
    )


class SceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
    )

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # Robot
    robot = TASER_CONFIG_USD.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class TaserBaseEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the TASER robot environment."""

    scene = SceneCfg(num_envs=1, env_spacing=3)
    actions = ActionsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 10
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 0.01
        self.sim.render_interval = 5
