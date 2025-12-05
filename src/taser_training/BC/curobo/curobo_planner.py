from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import carb
import isaacsim.core.utils.prims as prim_utils
import numpy as np
import torch
from curobo.geom.types import WorldConfig
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import log_info, log_warn, setup_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from pxr import UsdGeom
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
    target_position: np.ndarray  # (num_envs, 3)
    start_cfg: np.ndarray  # (num_envs, num_joints)
    joint_positions: np.ndarray  # (num_envs, num_time_steps, num_joints)
    joint_velocities: np.ndarray  # (num_envs, num_time_steps, num_joints)
    ee_positions: np.ndarray  # (num_envs, num_time_steps, 3)


class CuroboPlanner:
    def __init__(
        self,
        scene: Scene,
        robot: Articulation,
        interpolation_dt: float,
        env_origins: torch.Tensor,
        is_static_terrain: bool = True,
        device: torch.device = torch.device("cpu"),
    ):
        """
        cuRobo motion planner class.

        Args:
            env (MoleSimEnv): The simulation environment.
            interpolation_dt (float): The trajectory time step.
            env_origins (torch.Tensor): The (x, y, z) positions of the environments in the world.
        """

        self.stage = scene.stage
        self.robot = robot
        self.env_origins = env_origins
        self.num_envs = env_origins.shape[0]
        self.is_static_terrain = is_static_terrain
        self.device = device

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

    def plan(self, side: Literal["left", "right"]) -> list[CuroboEpisode]:
        """
        Plan a batch of cuRobo trajectories for the specified environment index.

        Args:
            env_idx (int): The index of the environment to plan for.
            start_cfg (torch.Tensor): The starting joint configuration of the robot. If None, uses a random valid configuration.
            targets (torch.Tensor): The target poses as a tensor of shape (N, 7). If None, samples random target poses.

        Returns:
            list[CuroboEpisode]: A list of cuRobo episodes containing the planned trajectories.
        """
        if side == "left":
            motion_gen = self.motion_gen_left
        else:
            motion_gen = self.motion_gen_right

        # Get cuRobo world configs with info about obstacles in the scene
        if not self.is_static_terrain:
            world_cfg = self._get_world_cfg_from_obstacles()
            motion_gen.update_world(world_cfg)

        # Get joint limits to sample a valid random starting state
        joint_limits = motion_gen.kinematics.get_joint_limits().position

        # Sample a valid random starting state
        is_valid_start_cfg = False
        while not is_valid_start_cfg:
            start_cfg = (
                torch.rand((self.num_envs, joint_limits.shape[1]), device=self.device)
                * (joint_limits[1] - joint_limits[0])
                + joint_limits[0]
            )
            for cfg in start_cfg:
                start_js = JointState.from_position(position=cfg)
                is_valid_start_cfg, _ = motion_gen.check_start_state(start_js)
                if not is_valid_start_cfg:
                    break

        cu_js = JointState.from_position(
            position=start_cfg,
            joint_names=motion_gen.kinematics.joint_names,
        ).get_ordered_joint_state(motion_gen.kinematics.joint_names)

        # Sample new valid random targets
        targets = self._sample_targets(side=side)

        # Create the cuRobo goal poses
        goal_poses = Pose(
            position=targets.clone(),
            quaternion=torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=self.device).expand(
                targets.shape[0], -1
            ),
        )

        # Compute curobo solution
        result = motion_gen.plan_batch_env(
            start_state=cu_js,
            goal_pose=goal_poses,
            plan_config=self.plan_config,
        )

        if not result:
            log_warn(f"Planning {side}: Failed.")
            return None
        elif torch.count_nonzero(result.success) == 0:
            log_warn(f"Planning {side}: Failed. Result status: {result.status}")
            return None

        # Get the successful paths of the sampled targets
        if self.num_envs > 1:
            trajectories = result.get_paths()
            successes = result.success
        else:
            trajectories = [result.get_interpolated_plan()]
            successes = [result.success]

        max_trajectory_length = max(
            [traj.position.shape[0] for traj in trajectories if traj is not None]
        )

        plans = CuroboEpisode(
            target_position=targets.cpu().numpy(),
            start_cfg=start_cfg.cpu().numpy(),
            joint_positions=start_cfg.unsqueeze(1)
            .expand((self.num_envs, max_trajectory_length, joint_limits.shape[1]))
            .cpu()
            .numpy(),
            joint_velocities=np.zeros(
                (self.num_envs, max_trajectory_length, joint_limits.shape[1])
            ),
            ee_positions=np.zeros((self.num_envs, max_trajectory_length, 3)),
        )
        for traj_idx, trajectory in enumerate(trajectories):
            if successes[traj_idx]:
                new_cmd_plan = motion_gen.get_full_js(trajectory)
                new_cmd_plan = new_cmd_plan.get_ordered_joint_state(
                    motion_gen.kinematics.joint_names
                )

                pos_np = new_cmd_plan.position.cpu().numpy()
                pos_len = pos_np.shape[0]
                plans.joint_positions[traj_idx, :pos_len] = pos_np
                if pos_len < plans.joint_positions.shape[1]:
                    plans.joint_positions[traj_idx, pos_len:] = np.repeat(
                        pos_np[-1][None, :],
                        plans.joint_positions.shape[1] - pos_len,
                        axis=0,
                    )

                vel_np = new_cmd_plan.velocity.cpu().numpy()
                vel_len = vel_np.shape[0]
                plans.joint_velocities[traj_idx, :vel_len] = vel_np

                ee_np = (
                    motion_gen.compute_kinematics(new_cmd_plan).ee_pos_seq.cpu().numpy()
                )
                ee_len = ee_np.shape[0]
                plans.ee_positions[traj_idx, :ee_len] = ee_np
                if ee_len < plans.ee_positions.shape[1]:
                    plans.ee_positions[traj_idx, ee_len:] = np.repeat(
                        ee_np[-1][None, :], plans.ee_positions.shape[1] - ee_len, axis=0
                    )

        return plans

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

    def _sample_targets(self, side: Literal["left", "right"]) -> torch.Tensor:
        num_envs = self.env_origins.shape[0]
        y_range = Y_RANGE_LEFT if side == "left" else Y_RANGE_RIGHT
        positions = None

        while positions is None:
            x_sign = torch.where(torch.rand(num_envs) > 0.5, 1.0, -1.0)
            x = x_sign * (torch.rand(num_envs) * (X_RANGE[1] - X_RANGE[0]) + X_RANGE[0])
            y = torch.rand(num_envs) * (y_range[1] - y_range[0]) + y_range[0]
            z = torch.rand(num_envs) * (Z_RANGE[1] - Z_RANGE[0]) + Z_RANGE[0]
            new_samples = torch.stack((x, y, z), dim=1)

            # Filter out samples that are in collision with the terrain
            if self.terrain_mesh is not None:
                # Check distance between new samples and the mesh, negative sign distance means a point is outside the mesh
                query_sample = new_samples.clone() + self.env_origins.cpu()
                distance = self.terrain_mesh.nearest.signed_distance(query_sample)
                if torch.all(distance <= -0.5):
                    positions = new_samples.clone()
            else:
                positions = new_samples.clone()

        return positions.to(self.env_origins.device)

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
