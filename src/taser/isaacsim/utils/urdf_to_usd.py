from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pathlib import Path

import omni.kit.commands  # type: ignore
from ament_index_python.packages import get_package_share_directory
from isaacsim.asset.importer.urdf import _urdf  # type: ignore

from taser.isaacsim.utils.extensions import enable_extensions

URDF_IN_PATH = str(
    Path(get_package_share_directory("taser_ros"))
    / "robot_description"
    / "urdf"
    / "taser.urdf"
)

USD_OUT_PATH = "/workspaces/taser/src/taser_ros/robot_description/usd/taser.usd"


def main():
    enable_extensions()

    # world = World(stage_units_in_meters=1.0, physics_dt=0.01)

    import_config = _urdf.ImportConfig()
    import_config.convex_decomp = False
    import_config.fix_base = False
    import_config.merge_fixed_joints = False
    import_config.make_default_prim = True
    import_config.self_collision = True
    import_config.distance_scale = 1
    import_config.density = 0.0

    # Parse the robot's URDF file to generate a robot model
    result, robot_model = omni.kit.commands.execute(
        "URDFParseFile",
        urdf_path=URDF_IN_PATH,
        import_config=import_config,
    )

    robot_model.root_link = "base_wrapper"

    # Update the joint drive parameters for better stiffness and damping
    for joint in robot_model.joints:
        robot_model.joints[joint].drive.strength = 0.0
        robot_model.joints[joint].drive.damping = 1e12 if "wheel" in joint else 1e8

    # Import the robot onto the current stage and retrieve its prim path
    result, _ = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=URDF_IN_PATH,
        urdf_robot=robot_model,
        import_config=import_config,
        dest_path=USD_OUT_PATH,
    )

    if result:
        print(f"Successfully saved USD to {USD_OUT_PATH}")

    simulation_app.close()


if __name__ == "__main__":
    main()
