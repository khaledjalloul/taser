import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
parser.add_argument("--cmd_gui", action="store_true", help="Start the command GUI")
args = parser.parse_args()

###############################################################

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from taser.isaacsim.utils.extensions import enable_extensions

enable_extensions()

###############################################################

import threading

import rclpy
from isaacsim.core.api import World

from taser.isaacsim.robot import TaserIsaacSimRobot
from taser.isaacsim.scene import set_up_scene
from taser.isaacsim.utils.occupancy_grid import IsaacSimOccupancyGridGenerator
from taser.isaacsim.utils.ros2_tf_publisher import set_up_omni_graph
from taser.ros.isaac.cmd_gui import start_cmd_gui


class TaserIsaacSim:
    def __init__(self):
        set_up_omni_graph()

        self.world = World()

        set_up_scene(scene=self.world.scene)

        self.robot = TaserIsaacSimRobot(
            position=(0.0, 0.0, 0.65),
            orientation=(1.0, 0.0, 0.0, 0.0),
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
    rclpy.init()

    if args.cmd_gui:
        threading.Thread(target=start_cmd_gui).start()

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
