import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D as Pose2DRos
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

from taser.common.datatypes import Pose, VelocityCommand
from taser.locomotion import DifferentialDriveKinematics
from taser.manipulation import IKManipulator
from taser.navigation import PolygonNavigator
from taser.ros.standalone.parameters import load_sim_parameters
from taser.ros.standalone.ros_node import TaserStandaloneRosNode


class TaserRvizSim:
    def __init__(self):
        self.node = TaserStandaloneRosNode(
            navigation_target_pose_cb=self.navigation_target_pose_cb,
            left_arm_velocity_cb=self.left_arm_velocity_cb,
            right_arm_velocity_cb=self.right_arm_velocity_cb,
        )

        params = load_sim_parameters(self.node)

        self._dt: float = params.general.dt
        self._current_velocity: float = 0.0
        self._path_plan: list[Pose] = []
        self._left_arm_desired_velocity: Vector3 = Vector3()
        self._right_arm_desired_velocity: Vector3 = Vector3()
        self._polygons = params.navigation.polygons

        self._manipulator = IKManipulator()

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

    def navigation_target_pose_cb(self, target: Pose2DRos):
        pose = self.node.get_robot_pose_from_sim()
        target_pose = Pose(x=target.x, y=target.y, rz=target.theta)

        self.node.spawn_target_in_sim(target_pose)

        self._path_plan = self._navigator.plan_path(pose, target_pose)

    def left_arm_velocity_cb(self, v_desired: Vector3):
        self._left_arm_desired_velocity = v_desired

    def right_arm_velocity_cb(self, v_desired: Vector3):
        self._right_arm_desired_velocity = v_desired

    def step(self):
        # Get wheel commands
        pose = self.node.get_robot_pose_from_sim()
        base_vel_cmd = VelocityCommand(0.0, 0.0)

        if self._path_plan:
            base_vel_cmd, reached = self._navigator.step(pose, self._current_velocity)
            self._current_velocity = base_vel_cmd.v
            if reached:
                self._path_plan = []

        wheel_velocities = self.diff_drive_kinematics.step(base_vel_cmd)

        # Get left arm joint commands
        left_joint_velocities = self._manipulator.get_dq_from_linear_v(
            v_desired=np.array(
                [
                    self._left_arm_desired_velocity.x,
                    self._left_arm_desired_velocity.y,
                    self._left_arm_desired_velocity.z,
                ]
            ),
            q_current=self.node.joint_positions.ordered,
            arm="left",
        )

        # Get right arm joint commands
        right_joint_velocities = self._manipulator.get_dq_from_linear_v(
            v_desired=np.array(
                [
                    self._right_arm_desired_velocity.x,
                    self._right_arm_desired_velocity.y,
                    self._right_arm_desired_velocity.z,
                ]
            ),
            q_current=self.node.joint_positions.ordered,
            arm="right",
        )

        # Publish all commands
        self.node._wheels_joint_velocity_pub.publish(
            Float64MultiArray(data=wheel_velocities)
        )
        self.node._left_arm_joint_velocity_pub.publish(
            Float64MultiArray(data=left_joint_velocities)
        )
        self.node._right_arm_joint_velocity_pub.publish(
            Float64MultiArray(data=right_joint_velocities)
        )

        # Update robot pose in simulation
        new_rz = pose.rz + base_vel_cmd.w * self._dt
        self.node.set_robot_pose_in_sim(
            Pose(
                x=pose.x + base_vel_cmd.v * math.cos(pose.rz) * self._dt,
                y=pose.y + base_vel_cmd.v * math.sin(pose.rz) * self._dt,
                z=pose.z,
                qw=math.cos(new_rz / 2.0),
                qz=math.sin(new_rz / 2.0),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    TaserRvizSim()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
