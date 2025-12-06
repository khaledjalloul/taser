from dataclasses import asdict, dataclass
from pathlib import Path

import carb
import isaacsim.core.utils.prims as prim_utils
import numpy as np
from curobo.geom.types import WorldConfig
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import log_info, log_warn, setup_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenResult
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from pxr import UsdGeom  # type: ignore
from trimesh import Trimesh

from taser_training.BC.curobo.config.curobo_planner_cfg import (
    CuroboMotionGenCfg,
    CuroboMotionGenPlanConfig,
)
from taser_training.BC.curobo.config.curobo_planner_cfg import (
    __file__ as CUROBO_CONFIG_PATH,
)

############################################################

X_RANGE = (0.2, 0.5)
Y_RANGE_LEFT = (0.0, 0.25)
Y_RANGE_RIGHT = (-0.25, 0.0)
Z_RANGE = (-0.4, 0.4)


@dataclass
class CuroboEpisode:
    start_cfg: np.ndarray  # (num_envs, 2, num_joints)
    joint_positions: np.ndarray  # (num_envs, num_time_steps, 2, num_joints)
    joint_velocities: np.ndarray  # (num_envs, num_time_steps, 2, num_joints)
    eef_positions: np.ndarray  # (num_envs, num_time_steps, 2, 3)
    target_position: np.ndarray  # (num_envs, 2, 3)
    episode_length: np.ndarray  # (num_envs, 2)


class CuroboPlanner:
    def __init__(
        self,
        scene: Scene,
        robot: Articulation,
        interpolation_dt: float,
        env_origins: np.ndarray,
        is_static_terrain: bool = True,
    ):
        self.stage = scene.stage
        self.robot = robot
        self.env_origins = env_origins
        self.num_envs = env_origins.shape[0]
        self.is_static_terrain = is_static_terrain

        # Load curobo robot configuration file
        robot_cfg_path = str(Path(CUROBO_CONFIG_PATH).parent / "robot_cfg.yaml")
        robot_cfg_left = load_yaml(robot_cfg_path)["robot_cfg_left"]
        robot_cfg_right = load_yaml(robot_cfg_path)["robot_cfg_right"]

        # Extract the terrain mesh to check for targets in collision while sampling
        self.terrain_mesh: Trimesh | None = None
        if prim_utils.is_prim_path_valid("/World/terrain/mesh"):
            self.terrain_mesh = self._extract_terrain_mesh_from_stage()

        # Set up logger
        setup_logger("error")
        log_info("Creating motion generator...")

        # Get cuRobo world config with info about obstacles in the scene
        world_cfgs = self._get_world_cfg_from_obstacles()

        # Instantiate curobo motion generator
        motion_gen_cfg = CuroboMotionGenCfg(interpolation_dt=interpolation_dt)
        self.plan_config = CuroboMotionGenPlanConfig()

        self.motion_gen_left = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg_left, world_cfgs, **asdict(motion_gen_cfg)
            )
        )
        self.motion_gen_right = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg_right, world_cfgs, **asdict(motion_gen_cfg)
            )
        )

    def plan(self) -> CuroboEpisode:
        start_cfg: dict[str, np.ndarray] = {"left": None, "right": None}
        cu_js: dict[str, JointState] = {"left": None, "right": None}
        targets = self._sample_targets()
        goal_poses: dict[str, Pose] = {"left": None, "right": None}
        result: dict[str, MotionGenResult] = {"left": None, "right": None}
        trajectories: dict[str, list[JointState]] = {"left": None, "right": None}
        successes: dict[str, list[bool]] = {"left": None, "right": None}

        if not self.is_static_terrain:
            world_cfg = self._get_world_cfg_from_obstacles()

        for side, motion_gen in zip(
            ["left", "right"],
            [self.motion_gen_left, self.motion_gen_right],
        ):
            if not self.is_static_terrain:
                motion_gen.update_world(world_cfg)

            # Get joint limits to sample a valid random starting state
            joint_limits = (
                motion_gen.kinematics.get_joint_limits().position.cpu().numpy()
            )

            # Sample a valid random starting state
            is_valid_start_cfg = False
            while not is_valid_start_cfg:
                start_cfg_side = (
                    np.random.random(
                        (self.num_envs, joint_limits.shape[1]),
                    )
                    * (joint_limits[1] - joint_limits[0])
                    + joint_limits[0]
                )
                for cfg in start_cfg_side:
                    start_js = JointState.from_position(
                        position=motion_gen.tensor_args.to_device(cfg)
                    )
                    is_valid_start_cfg, _ = motion_gen.check_start_state(start_js)
                    if not is_valid_start_cfg:
                        break
            start_cfg[side] = start_cfg_side

            cu_js[side] = JointState.from_position(
                position=motion_gen.tensor_args.to_device(start_cfg_side),
                joint_names=motion_gen.kinematics.joint_names,
            ).get_ordered_joint_state(motion_gen.kinematics.joint_names)

            # Create the cuRobo goal poses
            goal_poses[side] = Pose(
                position=motion_gen.tensor_args.to_device(targets[side]),
                quaternion=motion_gen.tensor_args.to_device(
                    [[1.0, 0.0, 0.0, 0.0]]
                ).repeat(targets[side].shape[0], 1),
            )

            # Compute curobo solution
            result[side] = motion_gen.plan_batch_env(
                start_state=cu_js[side],
                goal_pose=goal_poses[side],
                plan_config=self.plan_config,
            )

            if not result[side]:
                log_warn(f"Planning {side}: Failed.")
                return None
            elif np.count_nonzero(result[side].success.cpu()) == 0:
                log_warn(
                    f"Planning {side}: Failed. Result status: {result[side].status}"
                )
                return None

            # Get the successful paths of the sampled targets
            if self.num_envs > 1:
                trajectories[side] = result[side].get_paths()
                successes[side] = result[side].success
            else:
                trajectories[side] = [result[side].get_interpolated_plan()]
                successes[side] = [result[side].success]

        max_trajectory_length = max(
            [
                (traj.position.shape[0] if successes[side][traj_idx] else 0)
                for side in ["left", "right"]
                for traj_idx, traj in enumerate(trajectories[side])
            ]
        )

        start_cfg_stack = np.stack([c for c in start_cfg.values()], axis=1)
        episode = CuroboEpisode(
            start_cfg=start_cfg_stack,
            joint_positions=start_cfg_stack[:, None, :, :].repeat(
                max_trajectory_length, axis=1
            ),
            joint_velocities=np.zeros(
                (self.num_envs, max_trajectory_length, 2, joint_limits.shape[1])
            ),
            eef_positions=np.zeros((self.num_envs, max_trajectory_length, 2, 3)),
            target_position=np.stack([t for t in targets.values()], axis=1),
            episode_length=np.zeros((self.num_envs, 2)),
        )

        for side_idx, side in enumerate(["left", "right"]):
            motion_gen = (
                self.motion_gen_left if side == "left" else self.motion_gen_right
            )
            for env_idx, env_traj in enumerate(trajectories[side]):
                if successes[side][env_idx]:
                    new_cmd_plan = motion_gen.get_full_js(env_traj)
                    new_cmd_plan = new_cmd_plan.get_ordered_joint_state(
                        motion_gen.kinematics.joint_names
                    )

                    pos_np = new_cmd_plan.position.cpu().numpy()
                    pos_len = pos_np.shape[0]
                    episode.joint_positions[env_idx, :pos_len, side_idx] = pos_np
                    if pos_len < episode.joint_positions.shape[1]:
                        episode.joint_positions[env_idx, pos_len:, side_idx] = (
                            np.repeat(
                                pos_np[-1][None, :],
                                episode.joint_positions.shape[1] - pos_len,
                                axis=0,
                            )
                        )

                    vel_np = new_cmd_plan.velocity.cpu().numpy()
                    vel_len = vel_np.shape[0]
                    episode.joint_velocities[env_idx, :vel_len, side_idx] = vel_np

                    ee_np = (
                        motion_gen.compute_kinematics(new_cmd_plan)
                        .ee_pos_seq.cpu()
                        .numpy()
                    )
                    ee_len = ee_np.shape[0]
                    episode.eef_positions[env_idx, :ee_len, side_idx] = ee_np
                    if ee_len < episode.eef_positions.shape[1]:
                        episode.eef_positions[env_idx, ee_len:, side_idx] = np.repeat(
                            ee_np[-1][None, :],
                            episode.eef_positions.shape[1] - ee_len,
                            axis=0,
                        )

                    episode.episode_length[env_idx, side_idx] = pos_len

        return episode

    def _extract_terrain_mesh_from_stage(self) -> Trimesh:
        mesh_prim = self.stage.GetPrimAtPath("/World/terrain/mesh")
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

    def _get_world_cfg_from_obstacles(self) -> list[WorldConfig]:
        """
        Obtain world configurations containing information about obstacles for each robot environment.

        Args:
            stage: World stage
            num_envs: Number of environments
            env_origins: Positions (x, y, z) of the environment origins in the world frame

        Returns:
            list[WorldConfig]: World configurations with obstacles
        """
        world_cfgs = []
        for env_idx in range(self.env_origins.shape[0]):
            # Create reference frames at the position of the robots so they detect the terrain in the correct relative position
            frame_prim = self.stage.DefinePrim(
                f"/World/obstacle_frames/env_{env_idx}", "Xform"
            )
            frame_xform = UsdGeom.Xformable(frame_prim)
            frame_xform.SetXformOpOrder([])
            frame_xform.AddTranslateOp().Set(
                value=tuple(self.env_origins[env_idx].tolist())
            )

            usd_help = UsdHelper()
            usd_help.load_stage(self.stage)
            obstacles_ignore_substring = ["/World/obstacle_frames"]
            obstacles_ignore_substring.extend(
                [f"/World/envs/env_{env_idx}/taser", "/World/targets"]
            )

            world_cfgs.append(
                usd_help.get_obstacles_from_stage(
                    reference_prim_path=f"/World/obstacle_frames/env_{env_idx}",
                    ignore_substring=obstacles_ignore_substring,
                ).get_collision_check_world()
            )
        return world_cfgs

    def _sample_targets(self) -> dict[str, np.ndarray]:
        num_envs = self.env_origins.shape[0]
        positions = {"left": None, "right": None}

        for side in ["left", "right"]:
            y_range = Y_RANGE_LEFT if side == "left" else Y_RANGE_RIGHT
            while positions[side] is None:
                x_sign = np.where(np.random.rand(num_envs) > 0.5, 1.0, -1.0)
                x = x_sign * (
                    np.random.rand(num_envs) * (X_RANGE[1] - X_RANGE[0]) + X_RANGE[0]
                )
                y = np.random.rand(num_envs) * (y_range[1] - y_range[0]) + y_range[0]
                z = np.random.rand(num_envs) * (Z_RANGE[1] - Z_RANGE[0]) + Z_RANGE[0]
                new_samples = np.stack((x, y, z), axis=1)

                # Filter out samples that are in collision with the terrain
                if self.terrain_mesh is not None:
                    # Check distance between new samples and the mesh, negative sign distance means a point is outside the mesh
                    query_sample = new_samples + self.env_origins
                    distance = self.terrain_mesh.nearest.signed_distance(query_sample)
                    if np.all(distance <= -0.5):
                        positions[side] = new_samples
                else:
                    positions[side] = new_samples

        return positions
