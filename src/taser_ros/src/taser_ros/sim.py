import math

import rclpy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

from taser.common.datatypes import Pose
from taser.locomotion.differential_drive import DifferentialDriveKinematics
from taser.navigation import PolygonNavigator
from taser_ros.parameters import load_sim_parameters
from taser_ros.ros_node import RosNode


class RvizSim:
    def __init__(self):
        self.node = RosNode(
            navigation_target_pose_cb=self.set_navigation_target,
        )

        params = load_sim_parameters(self.node)

        self._dt: float = params.general.dt
        self._current_velocity: float = 0.0
        self._path_plan: list[Pose] = []
        self._polygons = params.navigation.polygons

        self._navigator = PolygonNavigator(
            workspace=params.navigation.workspace,
            polygons=self._polygons,
            v_max=params.navigation.v_max,
            w_max=params.navigation.w_max,
            wheel_base=params.locomotion.wheel_base,
            dt=params.general.dt,
            num_rrt_samples=params.navigation.num_rrt_samples,
            mpc_horizon=params.navigation.mpc_horizon,
        )

        self.diff_drive_kinematics = DifferentialDriveKinematics(
            wheel_base=params.locomotion.wheel_base,
            wheel_radius=params.locomotion.wheel_radius,
        )

        self.node.spawn_polygons_in_sim(params.navigation.polygons_sim)
        self.node.set_robot_pose_in_sim(params.general.initial_pose)

        self.node.create_timer(self._dt, self.step)
        rclpy.spin(self.node)

    def __del__(self):
        self.node.destroy_node()

    def set_navigation_target(self, target: Pose2D):
        pose = self.node.get_robot_pose_from_sim()
        target_pose = Pose(target.x, target.y, target.theta)

        self.node.spawn_target_in_sim(target_pose)

        self._path_plan = self._navigator.plan_path(pose, target_pose)

    def step(self):
        vel_cmd = None
        pose = self.node.get_robot_pose_from_sim()

        if self._path_plan:
            vel_cmd, reached = self._navigator.step(pose, self._current_velocity)
            self._current_velocity = vel_cmd.v
            if reached:
                self._path_plan = []

        if vel_cmd:
            wheel_velocities = self.diff_drive_kinematics.step(vel_cmd)
            msg = Float64MultiArray(data=wheel_velocities)
            self.node._wheels_joint_velocity_pub.publish(msg)

            self.node.set_robot_pose_in_sim(
                Pose(
                    pose.x + vel_cmd.v * math.cos(pose.theta) * self._dt,
                    pose.y + vel_cmd.v * math.sin(pose.theta) * self._dt,
                    pose.theta + vel_cmd.w * self._dt,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    RvizSim()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
