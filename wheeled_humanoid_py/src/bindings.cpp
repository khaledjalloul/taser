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
      .def("interpolate_path", &RRTPathPlanner::interpolate_path)
      .def("check_line_collision", &RRTPathPlanner::check_line_collision);

  // Types
  py::class_<Pose2D>(m, "Pose2D")
      .def(py::init<>()) // default constructor
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("theta", &Pose2D::theta);
}
