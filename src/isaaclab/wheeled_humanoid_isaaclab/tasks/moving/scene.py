import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg

from wheeled_humanoid_isaaclab.common import WHEELED_HUMANOID_CONFIG


class SceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg()
    )

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(0.75, 0.75, 0.75)
        )
    )

    # Robot
    robot = WHEELED_HUMANOID_CONFIG.replace(
        prim_path="{ENV_REGEX_NS}/wheeled_humanoid"
    )
