from dataclasses import dataclass

Vec2 = tuple[float, float]


@dataclass
class Pose:
    x: float
    y: float
    theta: float = 0.0  # yaw (rad)


@dataclass
class VelocityCommand:
    v: float  # linear m/s
    w: float  # angular rad/s
