import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument("--no-ros", action="store_true", help="Disable ROS2 publishing")
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from taser_sim.utils.extensions import enable_extensions

enable_extensions()

###############################################################

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from isaacsim.core.api import World
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from nav_msgs.msg import OccupancyGrid as OccupancyGridRos
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from taser.common.datatypes import TaserJointState
from taser.common.model import USD_PATH
from taser.locomotion import LocomotionPolicy
from taser.navigation import OccupancyGrid
from taser_sim.scene import set_up_scene
from taser_sim.utils.occupancy_grid import IsaacSimOccupancyGridGenerator
from taser_sim.utils.ros2_tf_publisher import add_tf_publisher, set_up_omni_graph
from taser_sim.utils.teleop import Teleop

NAME = "taser"
PRIM_PATH = f"/World/{NAME}"
SPAWN_POSITION_OFFSET = np.array([0.0, 0.0, 0.65])
ROS_PUBLISH_RATE = 5.0  # Hz


class TaserSimRosInterface(Node):
    def __init__(self):
        super().__init__("sim", namespace="taser")

        self.initial_pose = {
            "position": np.array([0.0, 0.0, 0.0]),
            "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.base_vel_cmd = np.zeros(3)
        self.joint_velocity_actions = TaserJointState()

        self.joint_state_pub = self.create_publisher(
            JointState, "/taser/sensors/joint_states", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/taser/sensors/odometry", 10)
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGridRos, "/taser/sensors/occupancy_grid", 10
        )

        self.create_subscription(
            Twist,
            "/taser/commands/base_velocity",
            self._base_velocity_cmd_cb,
            10,
        )
        self.create_subscription(
            JointState,
            "/taser/commands/joint_velocity",
            self._joint_velocity_cmd_cb,
            10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/taser/reset",
            self._reset_cb,
            10,
        )

    def _base_velocity_cmd_cb(self, msg: Twist):
        self.base_vel_cmd = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )

    def _joint_velocity_cmd_cb(self, msg: JointState):
        self.joint_velocity_actions = TaserJointState.construct_from(
            "ros", msg.velocity
        )

    def _reset_cb(self, msg: PoseWithCovarianceStamped):
        self.initial_pose = {
            "position": np.array(
                [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ]
            ),
            "orientation": np.array(
                [
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ]
            ),
        }
        self.needs_reset = True


class TaserIsaacSim(TaserSimRosInterface):
    def __init__(self):
        super().__init__()

        set_up_omni_graph()

        self.world = World()

        add_reference_to_stage(usd_path=str(USD_PATH), prim_path=PRIM_PATH)
        self.robot = Robot(
            name=NAME,
            prim_path=PRIM_PATH,
            position=self.initial_pose["position"] + SPAWN_POSITION_OFFSET,
            orientation=self.initial_pose["orientation"],
        )
        set_up_scene(scene=self.world.scene, robot=self.robot)

        add_tf_publisher(
            robot_name=NAME,
            target_prim=f"{PRIM_PATH}/base_link",
            tf_publisher_topic="/tf",
        )

        self.camera = Camera(
            name=f"{NAME}_camera",
            prim_path=f"{PRIM_PATH}/base_link/{NAME}_camera",
        )
        self.camera.set_focal_length(1.5)

        self.teleop = Teleop()
        self.locomotion_policy = LocomotionPolicy()
        self.occupancy_grid_generator = IsaacSimOccupancyGridGenerator()

        self.needs_reset = False
        self.step = 0

    def setup(self) -> None:
        self.world.add_physics_callback("taser_step", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size: float) -> None:
        if self.step == 0:
            self.occupancy_grid_generator.setup()
            self.target_prim: XFormPrim = self.world.stage.GetPrimAtPath(
                "/World/target"
            )
            self.step += 1
        elif self.needs_reset:
            self.world.reset(True)
            self.robot.set_world_pose(
                position=self.initial_pose["position"] + SPAWN_POSITION_OFFSET,
                orientation=self.initial_pose["orientation"],
            )
            self.needs_reset = False
            self.step = 0
        else:
            if (
                not args.no_ros
                and self.step % int(1.0 / (step_size * ROS_PUBLISH_RATE)) == 0
            ):
                self._publish_joint_states()
                self._publish_odometry()
                self._publish_occupancy_grid(
                    self.occupancy_grid_generator.get_occupancy_grid()
                )

            vel_cmd = self.teleop.get_command() * [
                self.locomotion_policy.v_max,
                self.locomotion_policy.v_max,
                self.locomotion_policy.w_max,
            ]
            if np.all(vel_cmd == 0):
                vel_cmd = self.base_vel_cmd

            _, quaternion_w = self.robot.get_world_pose()
            locomotion_action = self.locomotion_policy.step(
                joint_positions=TaserJointState.construct_from(
                    "isaac", self.robot.get_joint_positions()
                ),
                joint_velocities=TaserJointState.construct_from(
                    "isaac", self.robot.get_joint_velocities()
                ),
                base_quaternion_w=quaternion_w,
                base_linear_velocity_w=self.robot.get_linear_velocity(),
                base_angular_velocity_w=self.robot.get_angular_velocity(),
                base_target_planar_velocity_b=vel_cmd,
                dt=step_size,
            )
            # locomotion_action.left_arm = self.joint_velocity_actions.left_arm
            # locomotion_action.right_arm = self.joint_velocity_actions.right_arm

            action = ArticulationAction(joint_velocities=locomotion_action.to("isaac"))
            self.robot.apply_action(action)
            rclpy.spin_once(self, timeout_sec=0)
            self.step += 1

    def run(self) -> None:
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_stopped():
                self.needs_reset = True

    def _publish_joint_states(self) -> None:
        joint_state_msg = JointState()
        joint_state_msg.position = TaserJointState.construct_from(
            "isaac", self.robot.get_joint_positions()
        ).to("ros")
        joint_state_msg.velocity = TaserJointState.construct_from(
            "isaac", self.robot.get_joint_velocities()
        ).to("ros")
        self.joint_state_pub.publish(joint_state_msg)

    def _publish_odometry(self) -> None:
        position_w, orientation_w = self.robot.get_world_pose()
        position_w = position_w.astype(np.float64)
        orientation_w = orientation_w.astype(np.float64)
        linear_velocity_w = self.robot.get_linear_velocity().astype(np.float64)
        angular_velocity_w = self.robot.get_angular_velocity().astype(np.float64)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "World"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = position_w[0]
        odom_msg.pose.pose.position.y = position_w[1]
        odom_msg.pose.pose.position.z = position_w[2]
        odom_msg.pose.pose.orientation.w = orientation_w[0]
        odom_msg.pose.pose.orientation.x = orientation_w[1]
        odom_msg.pose.pose.orientation.y = orientation_w[2]
        odom_msg.pose.pose.orientation.z = orientation_w[3]
        odom_msg.twist.twist.linear.x = linear_velocity_w[0]
        odom_msg.twist.twist.linear.y = linear_velocity_w[1]
        odom_msg.twist.twist.linear.z = linear_velocity_w[2]
        odom_msg.twist.twist.angular.x = angular_velocity_w[0]
        odom_msg.twist.twist.angular.y = angular_velocity_w[1]
        odom_msg.twist.twist.angular.z = angular_velocity_w[2]

        self.odom_pub.publish(odom_msg)

    def _publish_occupancy_grid(self, occupancy_grid: OccupancyGrid) -> None:
        # Hide the robot from the occupancy grid to avoid self-collisions.
        position_w, _ = self.robot.get_world_pose()
        x_min = position_w[0] - 0.75
        x_max = position_w[0] + 0.75
        y_min = position_w[1] - 0.75
        y_max = position_w[1] + 0.75

        occupancy_grid.set((x_min, x_max, y_min, y_max), 0)

        # Publish the occupancy grid as a ROS message
        workspace = occupancy_grid.workspace

        msg = OccupancyGridRos()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "World"
        msg.info.resolution = occupancy_grid.cellsize
        msg.info.width = int(
            (workspace.x_max - workspace.x_min) / occupancy_grid.cellsize
        )
        msg.info.height = int(
            (workspace.y_max - workspace.y_min) / occupancy_grid.cellsize
        )
        # Origin must follow ROS notation, which is the bottom-left corner of the grid.
        msg.info.origin.position.x = workspace.x_min
        msg.info.origin.position.y = workspace.y_min
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy_grid.grid.astype(np.int8).flatten().tolist()
        self.occupancy_grid_pub.publish(msg)


def main():
    rclpy.init()

    sim = TaserIsaacSim()
    simulation_app.update()
    sim.world.reset()
    simulation_app.update()
    sim.setup()
    simulation_app.update()
    sim.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
