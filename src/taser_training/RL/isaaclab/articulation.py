import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim.converters import UrdfConverterCfg

from taser.common.model import URDF_PATH, USD_PATH

LEFT_ARM_JOINT_NAMES = [
    "base_link_left_arm_1_joint",
    "left_arm_1_left_arm_2_joint",
    "left_arm_2_left_arm_3_joint",
    "left_arm_3_left_arm_4_joint",
    "left_arm_4_left_arm_5_joint",
    "left_arm_5_left_arm_6_joint",
]

RIGHT_ARM_JOINT_NAMES = [
    "base_link_right_arm_1_joint",
    "right_arm_1_right_arm_2_joint",
    "right_arm_2_right_arm_3_joint",
    "right_arm_3_right_arm_4_joint",
    "right_arm_4_right_arm_5_joint",
    "right_arm_5_right_arm_6_joint",
]

GRIPPER_JOINT_NAMES = [
    "left_arm_6_left_arm_gripper1_joint",
    "left_arm_6_left_arm_gripper2_joint",
    "right_arm_6_right_arm_gripper1_joint",
    "right_arm_6_right_arm_gripper2_joint",
]

WHEEL_JOINT_NAMES = [
    "base_link_left_wheel_joint",
    "base_link_right_wheel_joint",
]

LOCK_JOINT_NAMES = [
    "base_link_front_lock_joint",
    "front_lock_support_joint",
    "base_link_back_lock_joint",
    "back_lock_support_joint",
]


INIT_STATE = ArticulationCfg.InitialStateCfg(
    joint_pos={
        **{joint: 0.0 for joint in LEFT_ARM_JOINT_NAMES},
        **{joint: 0.0 for joint in RIGHT_ARM_JOINT_NAMES},
        **{joint: 0.0 for joint in GRIPPER_JOINT_NAMES},
        **{joint: 0.0 for joint in LOCK_JOINT_NAMES},
        **{joint: 0.0 for joint in WHEEL_JOINT_NAMES},
    },
    pos=(0.0, 0.0, 0.65),
)

ACTUATORS = {
    "left_arm_joints": ImplicitActuatorCfg(
        joint_names_expr=LEFT_ARM_JOINT_NAMES,
        stiffness=None,
        damping=None,
    ),
    "right_arm_joints": ImplicitActuatorCfg(
        joint_names_expr=RIGHT_ARM_JOINT_NAMES,
        stiffness=None,
        damping=None,
    ),
    "gripper_joints": ImplicitActuatorCfg(
        joint_names_expr=GRIPPER_JOINT_NAMES,
        stiffness=None,
        damping=None,
    ),
    "wheel_joints": ImplicitActuatorCfg(
        joint_names_expr=WHEEL_JOINT_NAMES,
        stiffness=None,
        damping=None,
    ),
    "lock_joints": ImplicitActuatorCfg(
        joint_names_expr=LOCK_JOINT_NAMES,
        stiffness=None,
        damping=None,
    ),
}

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
    init_state=INIT_STATE,
    actuators=ACTUATORS,
)

TASER_CONFIG_USD = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH.resolve()),
    ),
    init_state=INIT_STATE,
    actuators=ACTUATORS,
)

TASER_CONFIG_FIXED_BASE_USD = TASER_CONFIG_USD.replace(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH.resolve()),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True,
        ),
    ),
)
