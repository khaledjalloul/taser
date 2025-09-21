from setuptools import find_packages, setup
import glob

package_name = "taser_ros"

robot_description_files = glob.glob("robot_description/**/*", recursive=True)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/config.rviz",
                "config/sim_parameters.yaml",
                "config/controller_manager.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/sim.launch.yaml",
                "launch/rviz_control.launch.yaml",
                "launch/rviz.launch.yaml",
            ],
        ),
        (
            "share/" + package_name + "/robot_description/urdf",
            ["robot_description/urdf/robot.urdf"],
        ),
        (
            "share/" + package_name + "/robot_description/urdf/xacro",
            [
                "robot_description/urdf/xacro/arm.xacro",
                "robot_description/urdf/xacro/base.xacro",
                "robot_description/urdf/xacro/wheel.xacro",
                "robot_description/urdf/xacro/ros2_control.xacro",
                "robot_description/urdf/xacro/robot.urdf.xacro",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kjalloul",
    maintainer_email="khaled.jalloul@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "sim = taser_ros.sim:main",
            "cmd_gui = taser_ros.cmd_gui:main",
        ],
    },
)
