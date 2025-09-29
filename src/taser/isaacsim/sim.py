import argparse

parser = argparse.ArgumentParser(description="Isaac Sim Taser Simulation")
parser.add_argument(
    "--headless", action="store_true", help="Run simulation in headless mode"
)
args = parser.parse_args()

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import World

from taser.isaacsim.robot import TaserIsaacSimRobot
from taser.isaacsim.scene import set_up_scene
from taser.isaacsim.utils.extensions import enable_extensions
from taser.isaacsim.utils.occupancy_grid import (
    get_occupancy_grid,
    set_up_occupancy_grid_generator,
)
from taser.isaacsim.utils.ros2_tf_publisher import set_up_omni_graph


def main():
    enable_extensions()

    # Set up ROS 2 bridge omni graph
    set_up_omni_graph()

    # Create robot instance
    robot = TaserIsaacSimRobot(
        position=(0.0, 0.0, 0.65),
        orientation=(1.0, 0.0, 0.0, 0.0),
    )

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=0.01,
        rendering_dt=0.05,
    )

    set_up_scene(world=world, robot=robot)
    world.reset()

    step = 0
    occupancy_grid = None

    while simulation_app.is_running():
        if world.is_playing():
            if step == 0:
                generator = set_up_occupancy_grid_generator()

            occupancy_grid = get_occupancy_grid(generator, plot=False)
            robot.step(dt=world.get_physics_dt(), occupancy_grid=occupancy_grid)

            step += 1

        if world.is_stopped():
            step = 0

        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
