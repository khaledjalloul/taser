from pathlib import Path

import isaaclab.sim as sim_utils
from ament_index_python.packages import get_package_share_directory
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim.converters import UrdfConverterCfg

URDF_PATH = (
    Path(get_package_share_directory("taser_ros"))
    / "robot_description"
    / "urdf"
    / "taser.urdf"
)

TASER_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(URDF_PATH.resolve()),
        fix_base=False,
        merge_fixed_joints=False,
        self_collision=True,
        root_link_name="base_wrapper",
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.1)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "base_left_arm_shoulder_joint": 0.0,
            "left_arm_1_left_arm_2_joint": 0.0,
            "left_arm_2_left_arm_3_joint": 0.0,
            "base_right_arm_shoulder_joint": 0.0,
            "right_arm_1_right_arm_2_joint": 0.0,
            "right_arm_2_right_arm_3_joint": 0.0,
            "base_left_wheel_joint": 0.0,
            "base_right_wheel_joint": 0.0,
        },
        pos=(0.0, 0.0, 0.0),
    ),
    actuators={
        "left_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_left_arm_shoulder_joint",
                "left_arm_1_left_arm_2_joint",
                "left_arm_2_left_arm_3_joint",
            ],
            stiffness=0.1,
            damping=0.1,
        ),
        "right_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_right_arm_shoulder_joint",
                "right_arm_1_right_arm_2_joint",
                "right_arm_2_right_arm_3_joint",
            ],
            stiffness=0.1,
            damping=0.1,
        ),
        "wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=["base_left_wheel_joint", "base_right_wheel_joint"],
            stiffness=0.1,
            damping=0.1,
        ),
    },
)
