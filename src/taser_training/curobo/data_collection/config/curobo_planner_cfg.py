from dataclasses import MISSING, dataclass, field

from curobo.geom.sdf.world import CollisionCheckerType


@dataclass
class CuroboCollisionCache:
    obb: int = 30  # Number of obstacle cuboids
    mesh: int = 10  # Number of obstacle meshes


@dataclass
class CuroboMotionGenCfg:
    """Motion generation params and configuration for the cuRobo planner"""

    interpolation_dt: float = MISSING

    position_threshold: float = 0.05
    # position_threshold: float = 0.2
    # rotation_threshold: float = 100  # Ignore orientation
    collision_activation_distance: float = 0.025

    # Collision Check Options
    collision_checker_type: CollisionCheckerType = CollisionCheckerType.MESH
    collision_cache: CuroboCollisionCache = field(
        default_factory=lambda: CuroboCollisionCache()
    )
