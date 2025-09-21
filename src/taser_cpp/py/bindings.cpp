#include <taser_cpp/robot.hpp>

#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/detail/common.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace taser_cpp;

PYBIND11_MODULE(taser_cpp, m) {

  // Locomotion Submodule
  py::module_ locomotion =
      m.def_submodule("locomotion", "Logic for the robot locomotion");

  // Locomotion Kinematics
  py::class_<locomotion::Kinematics>(locomotion, "Kinematics")
      .def(py::init<double, double, double>(), py::arg("L"),
           py::arg("wheel_radius"), py::arg("dt"))
      .def("set_base_velocity", &locomotion::Kinematics::set_base_velocity,
           py::arg("v"), py::arg("omega"))
      .def("set_wheel_velocities",
           &locomotion::Kinematics::set_wheel_velocities, py::arg("v_l"),
           py::arg("v_r"))
      .def("step", &locomotion::Kinematics::step, py::arg("dt"))
      .def_readwrite("pose", &locomotion::Kinematics::pose)
      .def_readwrite("base_velocity", &locomotion::Kinematics::base_velocity)
      .def_readwrite("v_l", &locomotion::Kinematics::v_l)
      .def_readwrite("v_r", &locomotion::Kinematics::v_r);

  // Manipulation Submodule
  py::module_ manipulation =
      m.def_submodule("manipulation", "Logic for the robot arms");

  // Manipulation Controller
  py::class_<manipulation::Controller>(manipulation, "Controller")
      .def(py::init<double>(), py::arg("kp"))
      .def("step", &manipulation::Controller::step, py::arg("x0"),
           py::arg("x_ss"));

  // Manipulation Kinematics
  py::class_<manipulation::Kinematics>(manipulation, "Kinematics")
      .def(py::init<std::string>(), py::arg("name"))
      .def("get_end_effector_pose",
           &manipulation::Kinematics::get_end_effector_pose, py::arg("TBE"))
      .def("get_end_effector_twist",
           &manipulation::Kinematics::get_end_effector_twist, py::arg("tfs"),
           py::arg("dq"))
      .def("solve_ik_for_joint_positions",
           &manipulation::Kinematics::solve_ik_for_joint_positions,
           py::arg("p_desired"), py::arg("tfs"))
      .def("solve_ik_for_joint_velocities",
           &manipulation::Kinematics::solve_ik_for_joint_velocities,
           py::arg("w_desired"), py::arg("tfs"));

  // Navigation Submodule
  py::module_ navigation =
      m.def_submodule("navigation", "Logic for the robot navigation");

  // Navigation Controller
  py::class_<navigation::Controller>(navigation, "Controller")
      .def(py::init<double, int, double, double>(), py::arg("dt"), py::arg("N"),
           py::arg("v_max"), py::arg("omega_max"))
      .def("step", &navigation::Controller::step, py::arg("x0"),
           py::arg("x_ref"), py::arg("u_ref"))
      .def_readwrite("N", &navigation::Controller::N);

  // Navigation RRT* Path Planner
  py::class_<navigation::PathPlanner>(navigation, "PathPlanner")
      .def(py::init<int, double, double, double, navigation::Dimensions>(),
           py::arg("num_samples"), py::arg("dt"), py::arg("L"),
           py::arg("desired_velocity"), py::arg("dim"))
      .def("set_obstacles", &navigation::PathPlanner::set_obstacles,
           py::arg("obstacles"))
      .def("generate_path", &navigation::PathPlanner::generate_path,
           py::arg("start"), py::arg("goal"))
      .def("sample_new_point", &navigation::PathPlanner::sample_new_point,
           py::arg("points"), py::arg("parent_idxs"), py::arg("distances"),
           py::arg("sample"))
      .def("sample_path", &navigation::PathPlanner::sample_path,
           py::arg("dubins_path"))
      .def("get_velocity_profile",
           &navigation::PathPlanner::get_velocity_profile, py::arg("path"))
      .def("check_collision", &navigation::PathPlanner::check_collision,
           py::arg("segment"))
      .def("create_halton_sample",
           &navigation::PathPlanner::create_halton_sample, py::arg("index"))
      .def("get_nearest_neighbors",
           &navigation::PathPlanner::get_nearest_neighbors, py::arg("new_pt"),
           py::arg("points"), py::arg("radius"));

  // Navigation Types
  py::class_<navigation::Dimensions>(navigation, "Dimensions")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("x_min"),
           py::arg("x_max"), py::arg("y_min"), py::arg("y_max"))
      .def_readwrite("x_min", &navigation::Dimensions::x_min)
      .def_readwrite("x_max", &navigation::Dimensions::x_max)
      .def_readwrite("y_min", &navigation::Dimensions::y_min)
      .def_readwrite("y_max", &navigation::Dimensions::y_max);

  py::enum_<navigation::Direction>(navigation, "Direction")
      .value("LEFT", navigation::Direction::LEFT)
      .value("RIGHT", navigation::Direction::RIGHT)
      .export_values();

  py::class_<navigation::Circle>(navigation, "Circle")
      .def(py::init<Pose2D, double>(), py::arg("center"), py::arg("radius"))
      .def(py::init<Pose2D, double, navigation::Direction>(), py::arg("center"),
           py::arg("radius"), py::arg("direction"))
      .def_readwrite("center", &navigation::Circle::center)
      .def_readwrite("radius", &navigation::Circle::radius)
      .def_readwrite("direction", &navigation::Circle::direction);

  py::class_<navigation::Arc>(navigation, "Arc")
      .def(py::init<>())
      .def(py::init<Pose2D, Pose2D, Pose2D, navigation::Direction, double,
                    double>(),
           py::arg("start"), py::arg("end"), py::arg("center"),
           py::arg("direction"), py::arg("radius"), py::arg("angle"))
      .def_readwrite("start", &navigation::Arc::start)
      .def_readwrite("end", &navigation::Arc::end)
      .def_readwrite("center", &navigation::Arc::center)
      .def_readwrite("direction", &navigation::Arc::direction)
      .def_readwrite("radius", &navigation::Arc::radius)
      .def_readwrite("angle", &navigation::Arc::angle)
      .def_readwrite("length", &navigation::Arc::length);

  py::class_<navigation::Line>(navigation, "Line")
      .def(py::init<>())
      .def(py::init<Pose2D, Pose2D>(), py::arg("start"), py::arg("end"))
      .def_readwrite("start", &navigation::Line::start)
      .def_readwrite("end", &navigation::Line::end)
      .def_readwrite("length", &navigation::Line::length);

  py::class_<navigation::DubinsSegment>(navigation, "DubinsSegment")
      .def(py::init<>())
      .def(py::init<navigation::Arc, navigation::Line>(), py::arg("arc"),
           py::arg("line"))
      .def_readwrite("arc", &navigation::DubinsSegment::arc)
      .def_readwrite("line", &navigation::DubinsSegment::line)
      .def_readwrite("length", &navigation::DubinsSegment::length);

  // Navigation Utils
  navigation.def("get_dubins_segment", &navigation::get_dubins_segment,
                 py::arg("start"), py::arg("goal"), py::arg("radius"));

  navigation.def("get_euclidean_distance", &navigation::get_euclidean_distance,
                 py::arg("a"), py::arg("b"));

  navigation.def("get_minimum_turning_radius",
                 &navigation::get_minimum_turning_radius,
                 py::arg("wheel_navigation"), py::arg("max_steering_angle"));

  navigation.def("get_tangent", &navigation::get_tangent, py::arg("circle"),
                 py::arg("target"));

  navigation.def("get_turning_circles", &navigation::get_turning_circles,
                 py::arg("pose"), py::arg("radius"));

  // Main module

  // Robot
  py::class_<RobotConfig>(m, "RobotConfig")
      .def(py::init<>())
      .def(py::init<double, double, double, double, int, double, int,
                    navigation::Dimensions>(),
           py::arg("dt"), py::arg("arm_controller_kp"), py::arg("base_L"),
           py::arg("base_wheel_radius"), py::arg("base_mpc_horizon"),
           py::arg("base_velocity"), py::arg("base_rrt_num_samples"),
           py::arg("base_rrt_dim"))
      .def_readwrite("dt", &RobotConfig::dt)
      .def_readwrite("arm_controller_kp", &RobotConfig::arm_controller_kp)
      .def_readwrite("base_L", &RobotConfig::base_L)
      .def_readwrite("base_wheel_radius", &RobotConfig::base_wheel_radius)
      .def_readwrite("base_mpc_horizon", &RobotConfig::base_mpc_horizon)
      .def_readwrite("base_velocity", &RobotConfig::base_velocity)
      .def_readwrite("base_rrt_num_samples", &RobotConfig::base_rrt_num_samples)
      .def_readwrite("base_rrt_dim", &RobotConfig::base_rrt_dim);

  py::class_<Robot>(m, "Robot")
      .def(py::init<RobotConfig>(), py::arg("config"))
      .def("move_arm_step", &Robot::move_arm_step, py::arg("arm_name"),
           py::arg("desired_position"), py::arg("tfs"))
      .def("plan_path", &Robot::plan_path, py::arg("goal"))
      .def("move_base_step", &Robot::move_base_step);

  // Types
  py::class_<Pose2D>(m, "Pose2D")
      .def(py::init<>())
      .def(py::init<double>(), py::arg("x"))
      .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
           py::arg("theta"))
      .def("list", &Pose2D::list)
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("theta", &Pose2D::theta);

  py::class_<BaseVelocity>(m, "BaseVelocity")
      .def(py::init<>())
      .def(py::init<double, double>(), py::arg("v"), py::arg("omega"))
      .def("list", &BaseVelocity::list)
      .def_readwrite("v", &BaseVelocity::v)
      .def_readwrite("omega", &BaseVelocity::omega);
}
