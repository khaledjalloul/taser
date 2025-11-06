import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.core.prims import RigidPrim


def set_up_scene(scene: Scene, robot: Robot):
    scene.add_default_ground_plane()

    prim_utils.create_prim(
        "/World/Light",
        "DomeLight",
        # position=np.array([1.0, 1.0, 1.0]),
        attributes={
            # "inputs:radius": 0.01,
            "inputs:intensity": 1e3,
            "inputs:color": (1.0, 1.0, 1.0),
        },
    )

    nucleus_root_path = get_assets_root_path()
    environment_prim_path = "/World/Environment"

    num_pallets = 4
    offset = 3.0

    count = 0
    for x in range(-num_pallets // 2, num_pallets // 2 + 1):
        for y in range(-num_pallets // 2, num_pallets // 2 + 1):
            if x == 0 and y == 0:
                continue  # Skip the origin
            count += 1
            pallet_prim_path = f"{environment_prim_path}/Pallet_{count}"
            add_reference_to_stage(
                usd_path=nucleus_root_path + "/Isaac/Props/Pallet/o3dyn_pallet.usd",
                prim_path=pallet_prim_path,
            )
            prim = RigidPrim(
                prim_path=pallet_prim_path,
                name=f"Pallet_{count}",
                position=(x * offset, y * offset, 0.0),
            )
            prim.set_mass(100000)

    DynamicCuboid(
        prim_path="/World/target",
        position=np.array([2.45, 0.0, 1.0]),
        scale=np.array([0.4, 0.4, 0.4]),
        color=np.array([0.63, 0.0, 0.8]),
    )

    scene.add(robot)
