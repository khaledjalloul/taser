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
      .def(py::init<double>())
      .def("step", &ArmController::step);

  // Arm Kinematics
  py::class_<ArmKinematics>(m, "ArmKinematics")
      .def(py::init<std::string>())
      .def("get_end_effector_pose", &ArmKinematics::get_end_effector_pose)
      .def("get_end_effector_twist", &ArmKinematics::get_end_effector_twist)
      .def("solve_ik_for_joint_positions",
           &ArmKinematics::solve_ik_for_joint_positions)
      .def("solve_ik_for_joint_velocities",
           &ArmKinematics::solve_ik_for_joint_velocities);

  // Base Controller
  py::class_<BaseController>(m, "BaseController")
      .def(py::init<double, int, double, double>())
      .def("step", &BaseController::step);

  // Base Kinematics
  py::class_<BaseKinematics>(m, "BaseKinematics")
      .def(py::init<double, double, double>())
      .def("set_L", &BaseKinematics::set_L)
      .def("set_base_velocity", &BaseKinematics::set_base_velocity)
      .def("set_wheel_velocities", &BaseKinematics::set_wheel_velocities)
      .def("step", &BaseKinematics::step);

  // Robot
  py::class_<Robot>(m, "Robot")
      .def(py::init())
      .def("move_arm_step", &Robot::move_arm_step)
      .def("move_base_step", &Robot::move_base_step);

  // RRT* Path Planner
  py::class_<RRTPathPlanner>(m, "RRTPathPlanner")
      .def(py::init<int>())
      .def("set_obstacles", &RRTPathPlanner::set_obstacles)
      .def("generate_path", &RRTPathPlanner::generate_path)
      .def("sample_new_point", &RRTPathPlanner::sample_new_point)
      .def("interpolate_path", &RRTPathPlanner::interpolate_path)
      .def("check_line_collision", &RRTPathPlanner::check_line_collision)
      .def("get_dimensions", &RRTPathPlanner::get_dimensions)
      .def("create_halton_sample", &RRTPathPlanner::create_halton_sample)
      .def("get_euclidean_distance", &RRTPathPlanner::get_euclidean_distance)
      .def("get_nearest_neighbors", &RRTPathPlanner::get_nearest_neighbors);

  // Types
  py::class_<Pose2D>(m, "Pose2D")
      .def(py::init<>())
      .def(py::init<double>())
      .def(py::init<double, double>())
      .def(py::init<double, double, double>())
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("theta", &Pose2D::theta);

  py::class_<Dimensions>(m, "Dimensions")
      .def(py::init<>())
      .def(py::init<double, double, double, double>())
      .def_readwrite("x_min", &Dimensions::x_min)
      .def_readwrite("x_max", &Dimensions::x_max)
      .def_readwrite("y_min", &Dimensions::y_min)
      .def_readwrite("y_max", &Dimensions::y_max);
}
