import carb
import numpy as np
import torch
from curobo.geom.types import WorldConfig
from curobo.util.usd_helper import UsdHelper
from isaacsim.core.api.objects import VisualCuboid
from pxr import Usd, UsdGeom
from trimesh import Trimesh

X_RANGE = (-0.5, 0.5)
Y_RANGE_LEFT = (0.0, 0.25)
Y_RANGE_RIGHT = (-0.25, 0.0)
Z_RANGE = (-0.4, 0.4)


# def collect_observations(
#     env: MoleSimEnv, target_xyz_b: torch.Tensor, target_quat: torch.Tensor
# ) -> torch.Tensor:
#     """
#     Collect observations for the dataset in the format required for the GPT model.

#     Args:
#         env (MoleSimEnv): The simulation environment.
#         target_xyz_b (torch.Tensor): Target position in the robot base frame of shape (num_envs, 3).
#         target_quat (torch.Tensor): Target orientation as quaternion of shape (num_envs, 4).
#     """

#     ee_pos_b = env.robot_measurements.bucket_pos_w - env.robot_measurements.root_pos_w
#     joint_pos = env.robot_measurements.joint_pos
#     joint_vel = env.robot_measurements.joint_vel
#     return torch.cat(
#         [
#             target_xyz_b,
#             ee_pos_b,
#             joint_pos,
#             joint_vel,
#             target_quat[:, 0].unsqueeze(1),
#         ],
#         dim=1,
#     )


def extract_terrain_mesh_from_stage(stage: Usd.Stage) -> Trimesh:
    """
    Extract trimesh data of the environment terrain.

    Args:
        stage: World stage

    Returns:
        Trimesh: Mesh object of the terrain
    """

    mesh_prim = stage.GetPrimAtPath("/World/terrain/mesh")
    usd_mesh = UsdGeom.Mesh(mesh_prim)
    points = usd_mesh.GetPointsAttr().Get()  # Gf.Vec3fArray
    indices = usd_mesh.GetFaceVertexIndicesAttr().Get()  # List[int]
    counts = usd_mesh.GetFaceVertexCountsAttr().Get()  # List[int]

    # Convert points to numpy
    vertices = np.array([[p[0], p[1], p[2]] for p in points])

    # Convert face indices to triangles
    faces = []
    i = 0
    for count in counts:
        if count == 3:
            faces.append(indices[i : i + 3])
        elif count > 3:
            # You could triangulate polygons here
            carb.log_warn("Non-triangle face, skipping...")
        i += count
    faces = np.array(faces)

    return Trimesh(vertices=vertices, faces=faces)


def get_world_cfg_from_obstacles(
    stage: Usd.Stage, env_idx: int, env_origins: torch.Tensor
) -> list[WorldConfig]:
    """
    Obtain world configurations containing information about obstacles for each robot environment.

    Args:
        stage: World stage
        num_envs: Number of environments
        env_origins: Positions (x, y, z) of the environment origins in the world frame

    Returns:
        list[WorldConfig]: World configurations with obstacles
    """
    # Create reference frames at the position of the robots so they detect the terrain in the correct relative position
    frame_prim = stage.DefinePrim(f"/World/obstacle_frames/env_{env_idx}", "Xform")
    frame_xform = UsdGeom.Xformable(frame_prim)
    frame_xform.SetXformOpOrder([])
    frame_xform.AddTranslateOp().Set(value=tuple(env_origins[env_idx].tolist()))

    usd_help = UsdHelper()
    usd_help.load_stage(stage)
    obstacles_ignore_substring = ["/World/obstacle_frames"]
    obstacles_ignore_substring.extend(
        [f"/World/envs/env_{env_idx}/Robot", "/World/target_*"]
    )

    return usd_help.get_obstacles_from_stage(
        reference_prim_path=f"/World/obstacle_frames/env_{env_idx}",
        ignore_substring=obstacles_ignore_substring,
    ).get_collision_check_world()


def sample_target_poses(
    num_targets: int,
    root_pos_w: torch.Tensor,
    terrain_mesh: Trimesh = None,
    start_eef_pos_b: torch.Tensor = None,
) -> dict[torch.Tensor, torch.Tensor]:
    """
    Sample a batch of random valid target poses in a single environment.

    Args:
        num_targets (int): Number of targets to sample.
        root_pos_w (torch.Tensor): Root position of the current environment in world frame (3,).
        desired_pitch (float): Desired pitch angle for the target orientations.
        terrain_mesh (Trimesh, optional): Terrain mesh for collision checking. Defaults to None.
        is_mpc (bool, optional): Whether the sampling is for MPC (limits theta range). Defaults to False.
        start_eef_pos_b (torch.Tensor, optional): Starting end-effector position to avoid sampling too close (3,). Defaults to None.

    Returns:
        torch.Tensor: Sampled target poses of shape (num_targets, 7)
        torch.Tensor: Corresponding orientations without yaw of shape (num_targets, 4)
    """

    positions = torch.zeros((2, 0, 3), dtype=torch.float32)

    while positions.shape[1] < num_targets:
        num_new_targets = num_targets - positions.shape[1]

        x_left = torch.rand(num_new_targets) * (X_RANGE[1] - X_RANGE[0]) + X_RANGE[0]
        y_left = (
            torch.rand(num_new_targets) * (Y_RANGE_LEFT[1] - Y_RANGE_LEFT[0])
            + Y_RANGE_LEFT[0]
        )
        z_left = torch.rand(num_new_targets) * (Z_RANGE[1] - Z_RANGE[0]) + Z_RANGE[0]
        new_samples_left = torch.stack((x_left, y_left, z_left), dim=1)

        x_right = torch.rand(num_new_targets) * (X_RANGE[1] - X_RANGE[0]) + X_RANGE[0]
        y_right = (
            torch.rand(num_new_targets) * (Y_RANGE_RIGHT[1] - Y_RANGE_RIGHT[0])
            + Y_RANGE_RIGHT[0]
        )
        z_right = torch.rand(num_new_targets) * (Z_RANGE[1] - Z_RANGE[0]) + Z_RANGE[0]
        new_samples_right = torch.stack((x_right, y_right, z_right), dim=1)

        new_samples = torch.stack((new_samples_left, new_samples_right), dim=0)

        # Filter out samples that are in collision with the terrain
        if terrain_mesh is not None:
            # Check distance between new samples and the mesh, negative sign distance means a point is outside the mesh
            query_samples = (
                new_samples.clone() + root_pos_w.cpu()
            )  # Translate to world coordinates
            distances = terrain_mesh.nearest.signed_distance(query_samples)
            new_samples = new_samples[:, distances <= -0.5]

            # if start_eef_pos_b is not None:
            #     # Further filter samples that are too close to the start eef position
            #     dists_to_start = torch.linalg.vector_norm(
            #         new_samples - start_eef_pos_b.cpu().unsqueeze(0), dim=1
            #     )
            #     new_samples = new_samples[dists_to_start >= 5.0]

        positions = torch.cat((positions, new_samples), dim=1)

    orientations_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32)[
        None, None, :
    ].repeat(2, num_targets, 1)

    return torch.cat((positions, orientations_quat), dim=2).to(root_pos_w.device)


def visualize_targets(
    positions: torch.Tensor, quaternions: torch.Tensor, prim_path: str
):
    """
    Spawn visualization cubes at the target positions in the simulation.

    Args:
        positions (torch.Tensor): Positions of the targets (N, 3).
        quaternions (torch.Tensor): Orientations of the targets as quaternions (N, 4).
        prim_path (str): The prim path where the cubes will be spawned.
    """
    VisualCuboid(
        prim_path=f"{prim_path}_left",
        position=positions[0].cpu().numpy(),
        orientation=quaternions[0].cpu().numpy(),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0.63, 0.0, 0.8]),
    )
    VisualCuboid(
        prim_path=f"{prim_path}_right",
        position=positions[1].cpu().numpy(),
        orientation=quaternions[1].cpu().numpy(),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0.0, 0.63, 0.8]),
    )
