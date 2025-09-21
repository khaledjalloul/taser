import math

import rclpy

from taser.common.datatypes import Pose, VelocityCommand
from taser.locomotion.differential_drive import DifferentialDriveKinematics
from taser.navigation import PolygonNavigator
from taser_ros.ros_node import RosNode


class RvizSim:
    def __init__(self):
        self.node = RosNode()

        params = self.node.load_params()

        self._polygons = params.navigation.polygons

        self.navigator = PolygonNavigator(
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

        self._dt: float = params.general.dt
        self._pose: Pose = params.general.initial_pose

        self.node.spawn_polygons_in_sim(params.navigation.polygons_sim)
        self.node.set_robot_pose_in_sim(self._pose)

        self.node.create_timer(self._dt, self.step)
        rclpy.spin(self.node)

    def __del__(self):
        self.node.destroy_node()

    def step(self):
        # vel_cmd = self.navigator.step(self._pose)
        vel_cmd = VelocityCommand(1, 0.2)

        new_pose = Pose(
            self._pose.x + vel_cmd.v * math.cos(self._pose.theta) * self._dt,
            self._pose.y + vel_cmd.v * math.sin(self._pose.theta) * self._dt,
            self._pose.theta + vel_cmd.w * self._dt,
        )
        self._pose = new_pose
        self.node.set_robot_pose_in_sim(self._pose)


def main(args=None):
    rclpy.init(args=args)
    RvizSim()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
