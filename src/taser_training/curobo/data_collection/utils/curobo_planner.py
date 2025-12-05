import traceback
from dataclasses import asdict, dataclass

import isaacsim.core.utils.prims as prim_utils
import torch
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import log_info, log_warn, setup_logger
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation

from taser_training.curobo.data_collection.config.curobo_planner_cfg import (
    CuroboMotionGenCfg,
)
from taser_training.curobo.data_collection.utils.utils import (
    extract_terrain_mesh_from_stage,
    get_world_cfg_from_obstacles,
    sample_target_poses,
)
from curobo.rollout.cost.pose_cost import PoseCostMetric

############################################################


@dataclass
class CuroboEpisode:
    target_left_position: torch.Tensor
    target_left_quaternion: torch.Tensor
    target_right_position: torch.Tensor
    target_right_quaternion: torch.Tensor
    start_cfg: torch.Tensor
    joint_positions: list[list[float]]
    joint_velocities: list[list[float]]
    ee_positions: list[list[float]]
    lidar_point_cloud: list[torch.Tensor] | None
    height_map_point_cloud: list[torch.Tensor] | None


class CuroboPlanner:
    def __init__(
        self,
        scene: Scene,
        robot: Articulation,
        interpolation_dt: float,
        env_origins: torch.Tensor,
        num_plans_per_terrain: int,
        is_static_terrain: bool = True,
        device: torch.device = torch.device("cpu"),
    ):
        """
        cuRobo motion planner class.

        Args:
            env (MoleSimEnv): The simulation environment.
            interpolation_dt (float): The trajectory time step.
            env_origins (torch.Tensor): The (x, y, z) positions of the environments in the world.
            num_plans_per_terrain (int): The number of plans to generate for each unique terrain.
        """

        self.stage = scene.stage
        self.robot = robot
        self.env_origins = env_origins
        self.num_plans_per_terrain = num_plans_per_terrain
        self.is_static_terrain = is_static_terrain
        self.device = device

        # Load curobo robot configuration file
        robot_cfg_path = "/workspace/taser/src/taser_training/curobo/data_collection/config/robot_cfg.yaml"
        robot_cfg = load_yaml(robot_cfg_path)["robot_cfg"]

        # Extract the terrain mesh to check for targets in collision while sampling
        self.terrain_mesh = None
        if prim_utils.is_prim_path_valid("/World/terrain/mesh"):
            self.terrain_mesh = extract_terrain_mesh_from_stage(self.stage)

        # Set up logger
        setup_logger("info")
        log_info("Creating motion generator...")

        # Get cuRobo world config with info about obstacles in the scene
        world_cfg0 = get_world_cfg_from_obstacles(self.stage, 0, env_origins)

        # Instantiate curobo motion generator
        motion_gen_cfg = CuroboMotionGenCfg(interpolation_dt=interpolation_dt)
        self.motion_gen = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg, world_cfg0, **asdict(motion_gen_cfg)
            )
        )

        pose_cost_metric = PoseCostMetric(
            reach_partial_pose=True,
            reach_vec_weight=self.motion_gen.tensor_args.to_device([0, 0, 0, 1, 1, 1]),
        )
        self.motion_gen.update_pose_cost_metric(pose_cost_metric)

    def plan(
        self,
        env_idx: int,
        start_cfg: torch.Tensor = None,
        targets: torch.Tensor = None,
    ) -> list[CuroboEpisode]:
        """
        Plan a batch of cuRobo trajectories for the specified environment index.

        Args:
            env_idx (int): The index of the environment to plan for.
            start_cfg (torch.Tensor): The starting joint configuration of the robot. If None, uses a random valid configuration.
            targets (torch.Tensor): The target poses as a tensor of shape (N, 7). If None, samples random target poses.

        Returns:
            list[CuroboEpisode]: A list of cuRobo episodes containing the planned trajectories.
        """
        plans: list[CuroboEpisode] = []

        # Get cuRobo world configs with info about obstacles in the scene
        if not self.is_static_terrain:
            world_cfg = get_world_cfg_from_obstacles(
                self.stage, env_idx, self.env_origins
            )
            self.motion_gen.update_world(world_cfg)

        if start_cfg is None:
            # Get joint limits to sample a valid random starting state
            joint_limits = self.motion_gen.kinematics.get_joint_limits().position

            # Sample a valid random starting state
            is_valid_start_cfg = False
            while not is_valid_start_cfg:
                start_cfg = (
                    torch.rand(joint_limits.shape[1], device=self.device)
                    * (joint_limits[1] - joint_limits[0])
                    + joint_limits[0]
                )
                start_js = JointState.from_position(position=start_cfg)
                is_valid_start_cfg, _ = self.motion_gen.check_start_state(start_js)
                start_eef_pos_b = self.motion_gen.compute_kinematics(
                    start_js
                ).ee_pos_seq[0]
        else:
            start_js = JointState.from_position(position=start_cfg)
            start_eef_pos_b = self.motion_gen.compute_kinematics(start_js).ee_pos_seq[0]

        cu_js = (
            JointState.from_position(
                position=start_cfg,
                joint_names=self.motion_gen.kinematics.joint_names,
            )
            .get_ordered_joint_state(self.motion_gen.kinematics.joint_names)
            .unsqueeze(0)
        )

        if targets is None:
            # Sample new valid random targets
            targets = sample_target_poses(
                num_targets=self.num_plans_per_terrain,
                root_pos_w=self.env_origins[env_idx],
                terrain_mesh=self.terrain_mesh,
                start_eef_pos_b=start_eef_pos_b,
            )

        # Create the cuRobo goal poses
        goal_poses_left = Pose(
            position=targets[0, :, :3].clone(),
            quaternion=targets[0, :, 3:].clone(),
        )

        goal_poses_right = Pose(
            position=targets[1, :, :3].clone(),
            quaternion=targets[1, :, 3:].clone(),
        )

        # Compute curobo solution(s)
        if self.num_plans_per_terrain > 1:
            try:
                result = self.motion_gen.plan_batch(
                    start_state=cu_js.repeat_seeds(len(goal_poses_left)),
                    goal_pose=goal_poses_left,
                    # link_poses={"right_arm_eef": goal_poses_right},
                )
            except RuntimeError as e:
                log_warn(f"Env {env_idx}: Encountered error {e}. Skipping planning.")
                log_warn(traceback.format_exc())
                return []
        else:
            result = self.motion_gen.plan_single(
                start_state=cu_js,
                goal_pose=goal_poses_left,
                # link_poses={"right_arm_eef": goal_poses_right},
            )

        if not result or torch.count_nonzero(result.success) == 0:
            log_warn(
                f"Env {env_idx}: None of the targets could be reached. Skipping planning."
            )
            if self.num_plans_per_terrain == 1:
                log_warn(f"Result status: {result.status}")

            # TODO: Fix weird error that sometimes causes all environments to fail
            # For now breaking out of the loop at the first failure makes data collection faster
            return []

        # Filter out unsuccessful targets
        targets_left_success = targets[0, result.success]
        targets_right_success = targets[1, result.success]

        # Get the successful paths of the sampled targets
        trajectories = (
            result.get_successful_paths()
            if self.num_plans_per_terrain > 1
            else [result.get_interpolated_plan()]
        )

        # Convert to CuroboEpisode class and add to the list of plans
        for traj_idx, traj in enumerate(trajectories):
            new_cmd_plan = self.motion_gen.get_full_js(traj)
            new_cmd_plan = new_cmd_plan.get_ordered_joint_state(
                self.motion_gen.kinematics.joint_names
            )

            new_plan = CuroboEpisode(
                target_left_position=targets_left_success[traj_idx, :3].cpu().tolist(),
                target_left_quaternion=targets_left_success[traj_idx, 3:]
                .cpu()
                .tolist(),
                target_right_position=targets_right_success[traj_idx, :3]
                .cpu()
                .tolist(),
                target_right_quaternion=targets_right_success[traj_idx, 3:]
                .cpu()
                .tolist(),
                start_cfg=start_cfg.cpu().tolist(),
                joint_positions=new_cmd_plan.position.cpu().tolist(),
                joint_velocities=new_cmd_plan.velocity.cpu().tolist(),
                ee_positions=self.motion_gen.compute_kinematics(new_cmd_plan)
                .ee_pos_seq.cpu()
                .tolist(),
                lidar_point_cloud=None,
                height_map_point_cloud=None,
            )

            plans.append(new_plan)

        return plans
