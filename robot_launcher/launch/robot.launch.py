from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


RVIZ_LAUNCH_PATH = (
    Path(get_package_share_directory("robot_description"))
    / "launch" / "rviz_control.launch.py"
)


def generate_launch_description():
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(RVIZ_LAUNCH_PATH)),
    )

    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller_node",
        output="screen",
    )

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_launch,
            arm_controller_node,
            state_machine_node,
        ]
    )
