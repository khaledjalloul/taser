from typing import Literal

import carb
import numpy as np
import torch
from curobo.geom.types import WorldConfig
from curobo.util.usd_helper import UsdHelper
from pxr import Usd, UsdGeom
from trimesh import Trimesh

X_RANGE = (0.2, 0.5)
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


def sample_target(
    root_pos_w: torch.Tensor,
    side: Literal["left", "right"],
    terrain_mesh: Trimesh = None,
) -> dict[str, torch.Tensor]:
    y_range = Y_RANGE_LEFT if side == "left" else Y_RANGE_RIGHT
    position = None

    while position is None:
        x_sign = 1.0 if torch.rand(1) > 0.5 else -1.0
        x = x_sign * (torch.rand(1) * (X_RANGE[1] - X_RANGE[0]) + X_RANGE[0])
        y = torch.rand(1) * (y_range[1] - y_range[0]) + y_range[0]
        z = torch.rand(1) * (Z_RANGE[1] - Z_RANGE[0]) + Z_RANGE[0]
        new_sample = torch.cat((x, y, z), dim=0)

        # Filter out samples that are in collision with the terrain
        if terrain_mesh is not None:
            # Check distance between new samples and the mesh, negative sign distance means a point is outside the mesh
            query_sample = new_sample.clone() + root_pos_w.cpu()
            distance = terrain_mesh.nearest.signed_distance(query_sample)
            if distance <= -0.5:
                position = new_sample.clone()
        else:
            position = new_sample.clone()

    quaternion = torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32)
    return torch.cat((position, quaternion), dim=0).to(root_pos_w.device)
