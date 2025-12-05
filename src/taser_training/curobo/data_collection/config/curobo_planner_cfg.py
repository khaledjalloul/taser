from dataclasses import MISSING, dataclass, field

import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.rollout.cost.pose_cost import PoseCostMetric
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig


@dataclass
class CuroboCollisionCache:
    obb: int = 30  # Number of obstacle cuboids
    mesh: int = 10  # Number of obstacle meshes


@dataclass
class CuroboMotionGenCfg:
    """Motion generation params and configuration for the cuRobo planner"""

    interpolation_dt: float = MISSING

    position_threshold: float = 0.1
    collision_activation_distance: float = 0.025
    maximum_trajectory_dt: float = 0.5

    # Collision Check Options
    collision_checker_type: CollisionCheckerType = CollisionCheckerType.MESH
    collision_cache: CuroboCollisionCache = field(
        default_factory=lambda: CuroboCollisionCache()
    )


@dataclass
class CuroboMotionGenPlanConfig(MotionGenPlanConfig):
    """Motion generation plan configuration for the cuRobo planner"""

    pose_cost_metric: PoseCostMetric = field(
        default_factory=lambda: PoseCostMetric(
            reach_partial_pose=True,
            reach_vec_weight=torch.tensor([0, 0, 0, 1, 1, 1]),
        )
    )
