from dataclasses import dataclass, field

import numpy as np

Vec2 = tuple[float, float]


@dataclass
class Workspace:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def tuple(self) -> tuple[float, float, float, float]:
        return (self.x_min, self.x_max, self.y_min, self.y_max)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    # TODO: Add converter methods
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


@dataclass
class VelocityCommand:
    v: float  # linear m/s
    w: float  # angular rad/s

    def tuple(self) -> tuple[float, float]:
        return (self.v, self.w)


Polygon = list[Pose]


@dataclass
class TaserJointState:
    left_arm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    right_arm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wheels: np.ndarray = field(default_factory=lambda: np.zeros(2))

    @classmethod
    def from_ros(cls, ros_state: list[float]) -> "TaserJointState":
        left_arm = [ros_state[0], ros_state[1], ros_state[3]]
        right_arm = [ros_state[2], ros_state[4], ros_state[5]]
        wheels = ros_state[6:8]
        return TaserJointState(
            left_arm=np.array(left_arm),
            right_arm=np.array(right_arm),
            wheels=np.array(wheels),
        )

    @classmethod
    def from_isaac(cls, isaac_state: np.ndarray) -> "TaserJointState":
        left_arm = [isaac_state[0], isaac_state[4], isaac_state[6]]
        right_arm = [isaac_state[2], isaac_state[5], isaac_state[7]]
        wheels = [isaac_state[1], isaac_state[3]]
        return TaserJointState(
            left_arm=left_arm,
            right_arm=right_arm,
            wheels=wheels,
        )

    @property
    def ordered_rtb(self) -> np.ndarray:
        return np.concatenate([self.left_arm, self.right_arm, self.wheels])

    @property
    def ordered_isaac(self) -> np.ndarray:
        return np.array(
            [
                self.left_arm[0],
                self.wheels[0],
                self.right_arm[0],
                self.wheels[1],
                self.left_arm[1],
                self.right_arm[1],
                self.left_arm[2],
                self.right_arm[2],
            ]
        )
