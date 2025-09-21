from dataclasses import dataclass

from taser.common.datatypes import Pose


@dataclass
class TaserSimParameters:
    @dataclass
    class General:
        dt: float
        initial_pose: Pose

    @dataclass
    class Locomotion:
        wheel_base: float
        wheel_radius: float

    @dataclass
    class Manipulation:
        kp: float

    @dataclass
    class Navigation:
        workspace: tuple
        polygons: list
        v_max: float
        w_max: float
        num_rrt_samples: int
        mpc_horizon: int
        polygons_sim: list[dict]
        polygons: list[Pose]

    general: General
    locomotion: Locomotion
    manipulation: Manipulation
    navigation: Navigation
