import subprocess
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.converters import UrdfConverterCfg


XACRO_PATH = Path(get_package_share_directory(
    "wheeled_humanoid_ros")) / "urdf" / "robot.urdf.xacro"
URDF_PATH = Path("/tmp") / "wheeled_humanoid" / "robot.urdf"

URDF_PATH.parent.mkdir(parents=True, exist_ok=True)
subprocess.run(['xacro', XACRO_PATH, '-o', URDF_PATH], check=True)


WHEELED_HUMANOID_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(URDF_PATH.resolve()),
        fix_base=False,  # TODO: Change to False after fixing ground collision
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.1
            )
        )
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            'base_left_arm_1_joint': 0.0,
            'left_arm_1_left_arm_2_joint': 0.0,
            'left_arm_2_left_arm_3_joint': 0.0,
            'base_right_arm_1_joint': 0.0,
            'right_arm_1_right_arm_2_joint': 0.0,
            'right_arm_2_right_arm_3_joint': 0.0,
            'base_wheel_1_joint': 0.0,
            'base_wheel_2_joint': 0.0,
        },
        pos=(0.0, 0.0, 0.0),
    ),
    actuators={
        "left_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_left_arm_1_joint",
                "left_arm_1_left_arm_2_joint",
                "left_arm_2_left_arm_3_joint"
            ],
            stiffness=0.1,
            damping=0.1,
        ),
        "right_arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_right_arm_1_joint",
                "right_arm_1_right_arm_2_joint",
                "right_arm_2_right_arm_3_joint"
            ],
            stiffness=0.1,
            damping=0.1,
        ),
        "wheel_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_wheel_1_joint",
                "base_wheel_2_joint"
            ],
            stiffness=0.1,
            damping=0.1,
        ),
    },
)
