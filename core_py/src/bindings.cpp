#include <wheeled_humanoid/robot.hpp>

#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace wheeled_humanoid;

PYBIND11_MODULE(wheeled_humanoid_py, m) {

  // Arm Submodule
  py::module_ arm = m.def_submodule("arm", "Logic for the robot arms");

  // Arm Controller
  py::class_<arm::Controller>(arm, "Controller")
      .def(py::init<double>(), py::arg("kp"))
      .def("step", &arm::Controller::step, py::arg("x0"), py::arg("x_ss"));

  // Arm Kinematics
  py::class_<arm::Kinematics>(arm, "Kinematics")
      .def(py::init<std::string>(), py::arg("name"))
      .def("get_end_effector_pose", &arm::Kinematics::get_end_effector_pose,
           py::arg("TBE"))
      .def("get_end_effector_twist", &arm::Kinematics::get_end_effector_twist,
           py::arg("tfs"), py::arg("dq"))
      .def("solve_ik_for_joint_positions",
           &arm::Kinematics::solve_ik_for_joint_positions, py::arg("p_desired"),
           py::arg("tfs"))
      .def("solve_ik_for_joint_velocities",
           &arm::Kinematics::solve_ik_for_joint_velocities,
           py::arg("w_desired"), py::arg("tfs"));

  // Base Submodule
  py::module_ base = m.def_submodule("base", "Logic for the robot base");

  // Base Controller
  py::class_<base::Controller>(base, "Controller")
      .def(py::init<double, int, double, double>(), py::arg("dt"), py::arg("N"),
           py::arg("v_max"), py::arg("omega_max"))
      .def("step", &base::Controller::step, py::arg("x0"), py::arg("x_ref"),
           py::arg("u_ref"))
      .def_readwrite("N", &base::Controller::N);

  // Base Kinematics
  py::class_<base::Kinematics>(base, "Kinematics")
      .def(py::init<double, double, double>(), py::arg("L"),
           py::arg("wheel_radius"), py::arg("dt"))
      .def("set_base_velocity", &base::Kinematics::set_base_velocity,
           py::arg("v"), py::arg("omega"))
      .def("set_wheel_velocities", &base::Kinematics::set_wheel_velocities,
           py::arg("v_l"), py::arg("v_r"))
      .def("step", &base::Kinematics::step, py::arg("dt"))
      .def_readwrite("pose", &base::Kinematics::pose)
      .def_readwrite("base_velocity", &base::Kinematics::base_velocity);

  // Base RRT* Path Planner
  py::class_<base::PathPlanner>(base, "PathPlanner")
      .def(py::init<int, double, double>(), py::arg("num_samples"),
           py::arg("dt"), py::arg("L"))
      .def("set_obstacles", &base::PathPlanner::set_obstacles,
           py::arg("obstacles"))
      .def("generate_path", &base::PathPlanner::generate_path, py::arg("start"),
           py::arg("goal"))
      .def("sample_new_point", &base::PathPlanner::sample_new_point,
           py::arg("points"), py::arg("parent_idxs"), py::arg("distances"),
           py::arg("dim"), py::arg("sample"))
      .def("interpolate_path", &base::PathPlanner::interpolate_path,
           py::arg("path"), py::arg("desired_n_points"))
      .def("get_velocity_profile", &base::PathPlanner::get_velocity_profile,
           py::arg("path"))
      .def("create_halton_sample", &base::PathPlanner::create_halton_sample,
           py::arg("index"), py::arg("dim"))
      .def("get_nearest_neighbors", &base::PathPlanner::get_nearest_neighbors,
           py::arg("new_pt"), py::arg("points"), py::arg("radius"));

  // Base Types
  py::class_<base::Dimensions>(base, "Dimensions")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("x_min"),
           py::arg("x_max"), py::arg("y_min"), py::arg("y_max"))
      .def_readwrite("x_min", &base::Dimensions::x_min)
      .def_readwrite("x_max", &base::Dimensions::x_max)
      .def_readwrite("y_min", &base::Dimensions::y_min)
      .def_readwrite("y_max", &base::Dimensions::y_max);

  // Base Utils
  base.def("get_euclidean_distance", &base::get_euclidean_distance,
           py::arg("a"), py::arg("b"));

  // Main module

  // Robot
  py::class_<Robot>(m, "Robot")
      .def(py::init())
      .def("move_arm_step", &Robot::move_arm_step, py::arg("arm_name"),
           py::arg("desired_position"), py::arg("tfs"))
      .def("move_base_step", &Robot::move_base_step, py::arg("desired_pose"));

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
