from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # type: ignore
from isaacsim.asset.importer.urdf import _urdf  # type: ignore
from isaacsim.core.api import World
from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.stage import add_reference_to_stage

from taser.common.model import URDF_PATH, USD_PATH
from taser.isaacsim.utils.extensions import enable_extensions

DRIVE_DAMPING = 50


def main():
    enable_extensions()

    world = World(stage_units_in_meters=1.0, physics_dt=0.01)

    import_config = _urdf.ImportConfig()
    import_config.set_convex_decomp(False)
    import_config.set_fix_base(False)
    import_config.set_merge_fixed_joints(False)
    import_config.set_make_default_prim(True)
    import_config.set_self_collision(True)
    import_config.set_distance_scale(1)
    import_config.set_density(0.0)

    # Parse the robot's URDF file to generate a robot model
    result, robot_model = omni.kit.commands.execute(
        "URDFParseFile",
        urdf_path=str(URDF_PATH),
        import_config=import_config,
    )

    robot_model.root_link = "base_link"

    for joint in robot_model.joints.values():
        joint.drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)
        joint.drive.set_strength(50)

    # Import the robot onto the current stage and retrieve its prim path
    result, _ = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=str(URDF_PATH),
        urdf_robot=robot_model,
        import_config=import_config,
        dest_path=str(USD_PATH),
    )

    if result:
        print(f"Successfully saved USD to {str(USD_PATH)}")

    if not simulation_app.config["headless"]:
        scene: Scene = world.scene
        scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(USD_PATH), prim_path="/World/Taser")
        while simulation_app.is_running():
            world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
