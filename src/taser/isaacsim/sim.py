import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument(
    "--no_ros",
    action="store_true",
    help="Disable ROS 2 bridge and related functionality",
)
args = parser.parse_args()

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import rclpy
from isaacsim.core.api import World

from taser.isaacsim.robot import TaserIsaacSimRobot
from taser.isaacsim.scene import set_up_scene
from taser.isaacsim.utils.extensions import enable_extensions
from taser.isaacsim.utils.occupancy_grid import IsaacSimOccupancyGridGenerator
from taser.isaacsim.utils.ros2_tf_publisher import set_up_omni_graph


class TaserIsaacSim:
    def __init__(self):
        enable_extensions()

        # Set up ROS 2 bridge omni graph
        set_up_omni_graph()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=0.01,
            # rendering_dt=0.05,
        )

        set_up_scene(scene=self.world.scene)

        self.robot = TaserIsaacSimRobot(
            position=(0.0, 0.0, 0.65),
            orientation=(1.0, 0.0, 0.0, 0.0),
            ros_enabled=not args.no_ros,
        )

        self.occupancy_grid_generator = IsaacSimOccupancyGridGenerator()

        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        self.world.add_physics_callback("taser_step", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self.occupancy_grid_generator.setup()
            self.robot.initialize(self.occupancy_grid_generator.workspace)
            self.first_step = False
        elif self.needs_reset:
            self.world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            occupancy_grid = self.occupancy_grid_generator.get_occupancy_grid()
            self.robot.step(dt=step_size, occupancy_grid=occupancy_grid)

    def run(self) -> None:
        while simulation_app.is_running():
            self.world.step(render=True)
            if self.world.is_stopped():
                self.needs_reset = True


def main():
    if not args.no_ros:
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
