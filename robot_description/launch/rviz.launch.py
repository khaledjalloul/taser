from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


RVIZ_CONFIG_FILE = (
    Path(get_package_share_directory("robot_description")) /
    "config" / "config.rviz"
)

XACRO_FILE_PATH = (
    Path(get_package_share_directory("robot_description"))
    / "urdf" / "robot.urdf.xacro"
)

ROBOT_DESCRIPTION = ParameterValue(
    Command(["xacro ", str(XACRO_FILE_PATH)]))


def generate_launch_description():

    use_joint_state_publisher_gui = DeclareLaunchArgument(
        "use_joint_state_publisher_gui",
        default_value="true",
        description="Whether to use the joint state publisher GUI",
    )

    return LaunchDescription(
        [
            use_joint_state_publisher_gui,
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(RVIZ_CONFIG_FILE)],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": ROBOT_DESCRIPTION,
                    },
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("use_joint_state_publisher_gui")),
            ),
        ]
    )
