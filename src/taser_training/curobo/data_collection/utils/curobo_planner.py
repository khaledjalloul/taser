from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

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
    CuroboMotionGenPlanConfig,
)
from taser_training.curobo.data_collection.config.curobo_planner_cfg import (
    __file__ as CUROBO_CONFIG_PATH,
)
from taser_training.curobo.data_collection.utils.utils import (
    extract_terrain_mesh_from_stage,
    get_world_cfg_from_obstacles,
    sample_target,
)

############################################################


@dataclass
class CuroboEpisode:
    target_position: torch.Tensor
    target_quaternion: torch.Tensor
    start_cfg: torch.Tensor
    joint_positions: list[list[float]]
    joint_velocities: list[list[float]]
    ee_positions: list[list[float]]


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
        self.is_static_terrain = is_static_terrain
        self.device = device

        # Load curobo robot configuration file
        robot_cfg_path = str(Path(CUROBO_CONFIG_PATH).parent / "robot_cfg.yaml")
        robot_cfg_left = load_yaml(robot_cfg_path)["robot_cfg_left"]
        robot_cfg_right = load_yaml(robot_cfg_path)["robot_cfg_right"]

        # Extract the terrain mesh to check for targets in collision while sampling
        self.terrain_mesh = None
        if prim_utils.is_prim_path_valid("/World/terrain/mesh"):
            self.terrain_mesh = extract_terrain_mesh_from_stage(self.stage)

        # Set up logger
        setup_logger("error")
        log_info("Creating motion generator...")

        # Get cuRobo world config with info about obstacles in the scene
        world_cfg0 = get_world_cfg_from_obstacles(self.stage, 0, env_origins)

        # Instantiate curobo motion generator
        motion_gen_cfg = CuroboMotionGenCfg(interpolation_dt=interpolation_dt)
        self.plan_config = CuroboMotionGenPlanConfig()

        self.motion_gen_left = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg_left, world_cfg0, **asdict(motion_gen_cfg)
            )
        )
        self.motion_gen_right = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg_right, world_cfg0, **asdict(motion_gen_cfg)
            )
        )

    def plan(self, env_idx: int, side: Literal["left", "right"]) -> list[CuroboEpisode]:
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
            world_cfg = get_world_cfg_from_obstacles(
                self.stage, env_idx, self.env_origins
            )
            motion_gen.update_world(world_cfg)

        # Get joint limits to sample a valid random starting state
        joint_limits = motion_gen.kinematics.get_joint_limits().position

        # Sample a valid random starting state
        is_valid_start_cfg = False
        while not is_valid_start_cfg:
            start_cfg = (
                torch.rand(joint_limits.shape[1], device=self.device)
                * (joint_limits[1] - joint_limits[0])
                + joint_limits[0]
            )
            start_js = JointState.from_position(position=start_cfg)
            is_valid_start_cfg, _ = motion_gen.check_start_state(start_js)

        cu_js = (
            JointState.from_position(
                position=start_cfg,
                joint_names=motion_gen.kinematics.joint_names,
            )
            .get_ordered_joint_state(motion_gen.kinematics.joint_names)
            .unsqueeze(0)
        )

        # Sample new valid random targets
        target = sample_target(
            root_pos_w=self.env_origins[env_idx],
            terrain_mesh=self.terrain_mesh,
            side=side,
        )

        # Create the cuRobo goal poses
        goal_poses = Pose(
            position=target[:3].clone(),
            quaternion=target[3:].clone(),
        )

        # Compute curobo solution
        result = motion_gen.plan_single(
            start_state=cu_js,
            goal_pose=goal_poses,
            plan_config=self.plan_config,
        )

        if not result:
            log_warn(f"Env {env_idx} - side {side}: Failed.")
            return None
        elif not result.success:
            log_warn(
                f"Env {env_idx} - side {side}: Failed. Result status: {result.status}"
            )
            return None

        # Get the successful paths of the sampled targets
        trajectory = result.get_interpolated_plan()
        new_cmd_plan = motion_gen.get_full_js(trajectory)
        new_cmd_plan = new_cmd_plan.get_ordered_joint_state(
            motion_gen.kinematics.joint_names
        )

        return CuroboEpisode(
            target_position=target[:3].cpu().numpy(),
            target_quaternion=target[3:].cpu().numpy(),
            start_cfg=start_cfg.cpu().numpy(),
            joint_positions=new_cmd_plan.position.cpu().numpy(),
            joint_velocities=new_cmd_plan.velocity.cpu().numpy(),
            ee_positions=motion_gen.compute_kinematics(new_cmd_plan)
            .ee_pos_seq.cpu()
            .numpy(),
        )
