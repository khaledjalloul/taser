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

USD_PATH = (
    Path(get_package_share_directory("taser_ros"))
    / "robot_description"
    / "usd"
    / "taser.usd"
)

TASER_CONFIG_URDF = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(URDF_PATH.resolve()),
        fix_base=False,
        merge_fixed_joints=True,
        self_collision=True,
        root_link_name="base_link",
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None),
            target_type="velocity",
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "base_link_left_arm_shoulder_joint": 0.0,
            "left_arm_1_left_arm_2_joint": 0.0,
            "left_arm_2_left_arm_3_joint": 0.0,
            "base_link_right_arm_shoulder_joint": 0.0,
            "right_arm_1_right_arm_2_joint": 0.0,
            "right_arm_2_right_arm_3_joint": 0.0,
            "base_link_left_wheel_joint": 0.0,
            "base_link_right_wheel_joint": 0.0,
        },
        pos=(0.0, 0.0, 0.65),
    ),
    actuators={
        "left_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_link_left_arm_shoulder_joint",
                "left_arm_1_left_arm_2_joint",
                "left_arm_2_left_arm_3_joint",
            ],
            stiffness=None,
            damping=None,
        ),
        "right_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_link_right_arm_shoulder_joint",
                "right_arm_1_right_arm_2_joint",
                "right_arm_2_right_arm_3_joint",
            ],
            stiffness=None,
            damping=None,
        ),
        "wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_link_left_wheel_joint",
                "base_link_right_wheel_joint",
            ],
            stiffness=None,
            damping=None,
        ),
    },
)

TASER_CONFIG_USD = TASER_CONFIG_URDF.replace(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH.resolve()),
    ),
)
