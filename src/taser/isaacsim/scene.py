from isaacsim.core.api import World
from isaacsim.core.api.scenes import Scene

from taser.isaacsim.robot import TaserIsaacSimRobot


def set_up_scene(world: World, robot: TaserIsaacSimRobot) -> Scene:
    scene: Scene = world.scene
    scene.add_default_ground_plane()

    scene.add(robot)

    return scene
