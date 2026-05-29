from dataclasses import dataclass, field
from typing import ClassVar, Literal

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
    locks: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # front, front_support, back, back_support
    left_arm: np.ndarray = field(default_factory=lambda: np.zeros(6))
    left_gripper: np.ndarray = field(default_factory=lambda: np.zeros(2))
    right_arm: np.ndarray = field(default_factory=lambda: np.zeros(6))
    right_gripper: np.ndarray = field(default_factory=lambda: np.zeros(2))
    wheels: np.ndarray = field(default_factory=lambda: np.zeros(2))  # left, right

    rtb_indices: ClassVar["TaserJointState"]
    ros_indices: ClassVar["TaserJointState"]
    isaac_indices: ClassVar["TaserJointState"]

    @classmethod
    def construct_from(
        cls, format: Literal["rtb", "ros", "isaac"], state: np.ndarray
    ) -> "TaserJointState":
        if type(state) is not np.ndarray:
            state = np.array(state)
        indices: TaserJointState = cls.__dict__[f"{format}_indices"]
        return TaserJointState(
            left_arm=state[indices.left_arm],
            left_gripper=state[indices.left_gripper],
            right_arm=state[indices.right_arm],
            right_gripper=state[indices.right_gripper],
            wheels=state[indices.wheels],
            locks=state[indices.locks],
        )

    def to(self, format: Literal["rtb", "ros", "isaac"]) -> np.ndarray:
        indices: TaserJointState = self.__class__.__dict__[f"{format}_indices"]
        out = np.zeros(self.num_joints)
        out[indices.locks] = self.locks
        out[indices.left_arm] = self.left_arm
        out[indices.left_gripper] = self.left_gripper
        out[indices.right_arm] = self.right_arm
        out[indices.right_gripper] = self.right_gripper
        out[indices.wheels] = self.wheels
        return out

    @property
    def num_joints(self) -> int:
        return (
            self.locks.size
            + self.left_arm.size
            + self.left_gripper.size
            + self.right_arm.size
            + self.right_gripper.size
            + self.wheels.size
        )


TaserJointState.rtb_indices = TaserJointState(
    locks=np.array([0, 1, 2, 3]),
    left_arm=np.array([4, 5, 6, 7, 8, 9]),
    left_gripper=np.array([10, 11]),
    right_arm=np.array([12, 13, 14, 15, 16, 17]),
    right_gripper=np.array([18, 19]),
    wheels=np.array([20, 21]),
)

TaserJointState.ros_indices = TaserJointState(
    locks=np.array([0, 1, 2, 3]),
    left_arm=np.array([4, 5, 6, 7, 8, 9]),
    left_gripper=np.array([10, 11]),
    right_arm=np.array([12, 13, 14, 15, 16, 17]),
    right_gripper=np.array([18, 19]),
    wheels=np.array([20, 21]),
)

TaserJointState.isaac_indices = TaserJointState(
    locks=np.array([1, 7, 0, 6]),
    left_arm=np.array([2, 8, 10, 12, 14, 16]),
    left_gripper=np.array([18, 19]),
    right_arm=np.array([4, 9, 11, 13, 15, 17]),
    right_gripper=np.array([20, 21]),
    wheels=np.array([3, 5]),
)
