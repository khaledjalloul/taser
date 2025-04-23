from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


RVIZ_LAUNCH_PATH = (
    Path(get_package_share_directory("wheeled_humanoid_ros"))
    / "launch" / "rviz_control.launch.py"
)


def generate_launch_description():
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(RVIZ_LAUNCH_PATH)),
    )

    wheeled_humanoid_node = Node(
        package="wheeled_humanoid_ros",
        executable="wheeled_humanoid_ros_node",
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
            wheeled_humanoid_node,
            state_machine_node,
        ]
    )
