from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


RVIZ_LAUNCH_PATH = (
    Path(get_package_share_directory("wheeled_humanoid_ros"))
    / "launch" / "rviz.launch.py"
)

CONTROLLER_MANAGER_CONFIG = (
    Path(get_package_share_directory("wheeled_humanoid_ros"))
    / "config" / "controller_manager.yaml"
)


def generate_launch_description():
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(RVIZ_LAUNCH_PATH)),
        launch_arguments={'use_joint_state_publisher_gui': 'false'}.items(),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[str(CONTROLLER_MANAGER_CONFIG)],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_velocity_controller",
                   "right_arm_velocity_controller",
                   "wheels_velocity_controller",
                   "--param-file", str(CONTROLLER_MANAGER_CONFIG)],
    )

    return LaunchDescription(
        [
            rviz_launch,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            velocity_controller_spawner,
        ]
    )
