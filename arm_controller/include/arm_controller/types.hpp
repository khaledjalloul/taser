#pragma once

#include <eigen3/Eigen/Dense>

namespace arm_controller {

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

using Position = Eigen::Vector3d;
using Orientation = Eigen::Vector3d;

using Axis = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using Jacobian = Eigen::Matrix<double, 6, 3>;

using RobotJointState = Eigen::Vector<double, 10>;
using ArmJointState = Eigen::Vector3d;

struct Pose {
  Position position;
  Orientation orientation;
};

struct Transform {
  Position translation;
  Quaternion rotation;
};

} // namespace arm_controller