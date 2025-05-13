// bindings.cpp
#include <wheeled_humanoid/robot.hpp>

#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace wheeled_humanoid;

PYBIND11_MODULE(wheeled_humanoid_py, m) {
  // Arm Controller
  py::class_<ArmController>(m, "ArmController")
      .def(py::init<double>(), py::arg("kp"))
      .def("step", &ArmController::step, py::arg("x0"), py::arg("x_ss"));

  // Arm Kinematics
  py::class_<ArmKinematics>(m, "ArmKinematics")
      .def(py::init<std::string>(), py::arg("name"))
      .def("get_end_effector_pose", &ArmKinematics::get_end_effector_pose,
           py::arg("TBE"))
      .def("get_end_effector_twist", &ArmKinematics::get_end_effector_twist,
           py::arg("tfs"), py::arg("dq"))
      .def("solve_ik_for_joint_positions",
           &ArmKinematics::solve_ik_for_joint_positions, py::arg("p_desired"),
           py::arg("tfs"))
      .def("solve_ik_for_joint_velocities",
           &ArmKinematics::solve_ik_for_joint_velocities, py::arg("w_desired"),
           py::arg("tfs"));

  // Base Controller
  py::class_<BaseController>(m, "BaseController")
      .def(py::init<double, int, double, double>(), py::arg("dt"), py::arg("N"),
           py::arg("v_max"), py::arg("omega_max"))
      .def("step", &BaseController::step, py::arg("x0"), py::arg("x_ref"),
           py::arg("u_ref"))
      .def_readwrite("N", &BaseController::N);

  // Base Kinematics
  py::class_<BaseKinematics>(m, "BaseKinematics")
      .def(py::init<double, double, double>(), py::arg("L"),
           py::arg("wheel_radius"), py::arg("dt"))
      .def("set_base_velocity", &BaseKinematics::set_base_velocity,
           py::arg("v"), py::arg("omega"))
      .def("set_wheel_velocities", &BaseKinematics::set_wheel_velocities,
           py::arg("v_l"), py::arg("v_r"))
      .def("step", &BaseKinematics::step, py::arg("dt"))
      .def_readwrite("pose", &BaseKinematics::pose)
      .def_readwrite("base_velocity", &BaseKinematics::base_velocity);

  // Robot
  py::class_<Robot>(m, "Robot")
      .def(py::init())
      .def("move_arm_step", &Robot::move_arm_step, py::arg("arm_name"),
           py::arg("desired_position"), py::arg("tfs"))
      .def("move_base_step", &Robot::move_base_step, py::arg("desired_pose"));

  // RRT* Path Planner
  py::class_<RRTPathPlanner>(m, "RRTPathPlanner")
      .def(py::init<int, double, double>(), py::arg("num_samples"),
           py::arg("dt"), py::arg("L"))
      .def("set_obstacles", &RRTPathPlanner::set_obstacles,
           py::arg("obstacles"))
      .def("generate_path", &RRTPathPlanner::generate_path, py::arg("start"),
           py::arg("goal"))
      .def("sample_new_point", &RRTPathPlanner::sample_new_point,
           py::arg("points"), py::arg("parent_idxs"), py::arg("distances"),
           py::arg("dim"), py::arg("sample"))
      .def("interpolate_path", &RRTPathPlanner::interpolate_path,
           py::arg("path"), py::arg("desired_n_points"))
      .def("get_velocity_profile", &RRTPathPlanner::get_velocity_profile,
           py::arg("path"))
      .def("check_line_collision", &RRTPathPlanner::check_line_collision,
           py::arg("a"), py::arg("b"))
      .def("get_dimensions", &RRTPathPlanner::get_dimensions, py::arg("start"),
           py::arg("goal"))
      .def("create_halton_sample", &RRTPathPlanner::create_halton_sample,
           py::arg("index"), py::arg("dim"))
      .def("get_euclidean_distance", &RRTPathPlanner::get_euclidean_distance,
           py::arg("a"), py::arg("b"))
      .def("get_nearest_neighbors", &RRTPathPlanner::get_nearest_neighbors,
           py::arg("new_pt"), py::arg("points"), py::arg("radius"));

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

  py::class_<Dimensions>(m, "Dimensions")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("x_min"),
           py::arg("x_max"), py::arg("y_min"), py::arg("y_max"))
      .def_readwrite("x_min", &Dimensions::x_min)
      .def_readwrite("x_max", &Dimensions::x_max)
      .def_readwrite("y_min", &Dimensions::y_min)
      .def_readwrite("y_max", &Dimensions::y_max);
}
