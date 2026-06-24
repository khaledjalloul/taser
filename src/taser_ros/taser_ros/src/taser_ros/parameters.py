from dataclasses import dataclass
from typing import Any

from rcl_interfaces.msg import ParameterValue
from rclpy.node import Node

from taser.common.datatypes import Workspace


@dataclass
class TaserParameters:
    @dataclass
    class Manipulation:
        pass

    @dataclass
    class Navigation:
        workspace: Workspace
        goal_pose_tolerance: float
        wheel_base: float
        v_max: float
        w_max: float

    dt: float
    world_frame: str

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


def load_parameters(node: Node) -> TaserParameters:
    return TaserParameters(
        dt=load_parameter(node, "dt", 0.0).double_value,
        world_frame=load_parameter(node, "world_frame", "").string_value,
        manipulation=TaserParameters.Manipulation(),
        navigation=TaserParameters.Navigation(
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
            goal_pose_tolerance=load_parameter(
                node, "navigation.goal_pose_tolerance", 0.0
            ).double_value,
            wheel_base=load_parameter(node, "navigation.wheel_base", 0.0).double_value,
            v_max=load_parameter(node, "navigation.v_max", 0.0).double_value,
            w_max=load_parameter(node, "navigation.w_max", 0.0).double_value,
        ),
    )
