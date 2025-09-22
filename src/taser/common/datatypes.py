from dataclasses import dataclass

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
    x: float
    y: float
    theta: float = 0.0  # yaw (rad)

    def tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)


@dataclass
class VelocityCommand:
    v: float  # linear m/s
    w: float  # angular rad/s

    def tuple(self) -> tuple[float, float]:
        return (self.v, self.w)


@dataclass
class TaserJointState:
    ordered: list[float]
    left_arm: list[float]
    right_arm: list[float]
    wheels: list[float]

    @classmethod
    def from_ros(cls, ros_state: list[float]) -> "TaserJointState":
        left_arm = [ros_state[0], ros_state[4], ros_state[5]]
        right_arm = [ros_state[1], ros_state[6], ros_state[7]]
        wheels = ros_state[2:4]
        ordered = [*left_arm, *right_arm, *wheels]
        return TaserJointState(
            ordered=ordered,
            left_arm=left_arm,
            right_arm=right_arm,
            wheels=wheels,
        )
