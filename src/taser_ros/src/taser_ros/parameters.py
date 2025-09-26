from dataclasses import dataclass
from typing import Any

from rcl_interfaces.msg import ParameterValue
from rclpy.node import Node

from taser.common.datatypes import Pose2D, Workspace


@dataclass
class TaserSimParameters:
    @dataclass
    class General:
        dt: float
        initial_pose: Pose2D

    @dataclass
    class Locomotion:
        wheel_base: float
        wheel_radius: float

    @dataclass
    class Manipulation:
        kp: float

    @dataclass
    class Navigation:
        workspace: Workspace
        v_max: float
        w_max: float
        num_rrt_samples: int
        mpc_horizon: int
        polygons_sim: list[dict]
        polygons: list[list[Pose2D]]

    general: General
    locomotion: Locomotion
    manipulation: Manipulation
    navigation: Navigation


def load_parameter(
    node: Node,
    param_name: str,
    default_value: Any,
    declare: bool = True,
) -> ParameterValue:
    if declare:
        node.declare_parameter(param_name, default_value)
    return node.get_parameter(param_name).get_parameter_value()


def load_sim_parameters(node: Node) -> TaserSimParameters:
    params = TaserSimParameters(
        general=TaserSimParameters.General(
            dt=load_parameter(node, "dt", 0.0).double_value,
            initial_pose=Pose2D(
                load_parameter(node, "initial_pose.x", 0.0).double_value,
                load_parameter(node, "initial_pose.y", 0.0).double_value,
                load_parameter(node, "initial_pose.theta", 0.0).double_value,
            ),
        ),
        locomotion=TaserSimParameters.Locomotion(
            wheel_base=load_parameter(node, "locomotion.wheel_base", 0.0).double_value,
            wheel_radius=load_parameter(
                node, "locomotion.wheel_radius", 0.0
            ).double_value,
        ),
        manipulation=TaserSimParameters.Manipulation(
            kp=load_parameter(node, "manipulation.controller_kp", 0.0).double_value
        ),
        navigation=TaserSimParameters.Navigation(
            workspace=Workspace(
                x_min=load_parameter(
                    node, "navigation.workspace.x_min", 0.0
                ).double_value,
                x_max=load_parameter(
                    node, "navigation.workspace.x_max", 0.0
                ).double_value,
                y_min=load_parameter(
                    node, "navigation.workspace.y_min", 0.0
                ).double_value,
                y_max=load_parameter(
                    node, "navigation.workspace.y_max", 0.0
                ).double_value,
            ),
            v_max=load_parameter(node, "navigation.v_max", 0.0).double_value,
            w_max=load_parameter(node, "navigation.w_max", 0.0).double_value,
            num_rrt_samples=load_parameter(
                node, "navigation.num_rrt_samples", 0
            ).integer_value,
            mpc_horizon=load_parameter(node, "navigation.mpc_horizon", 0).integer_value,
            polygons_sim=[],
            polygons=[],
        ),
    )

    polygon_count = load_parameter(
        node, "navigation.polygons._polygon_count", 0
    ).integer_value

    for i in range(polygon_count):
        position = load_parameter(
            node, f"navigation.polygons.polygon_{i}.position", [0.0, 0.0]
        ).double_array_value
        size = load_parameter(
            node, f"navigation.polygons.polygon_{i}.size", [0.0, 0.0]
        ).double_array_value

        front_left = position[0] + size[0] / 2, position[1] - size[1] / 2
        front_right = position[0] + size[0] / 2, position[1] + size[1] / 2
        back_left = position[0] - size[0] / 2, position[1] - size[1] / 2
        back_right = position[0] - size[0] / 2, position[1] + size[1] / 2
        polygon_corners = (back_left, back_right, front_right, front_left)
        polygon = [Pose2D(*p) for p in polygon_corners]

        params.navigation.polygons_sim.append({"position": position, "size": size})
        params.navigation.polygons.append(polygon)

    return params
