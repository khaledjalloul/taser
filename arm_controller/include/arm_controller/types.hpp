#pragma once

#include <eigen3/Eigen/Dense>

namespace arm_controller {

using Vector = Eigen::VectorXd;
using Vector3 = Eigen::Vector3d;
using Matrix = Eigen::MatrixXd;

using Position = Vector3;
using Orientation = Vector3;

using Axis = Vector3;
using Quaternion = Eigen::Quaterniond;
using Jacobian = Eigen::Matrix<double, 6, 3>;

using RobotJointState = Eigen::Vector<double, 10>;
using ArmJointState = Vector3;

struct Pose {
  Position position;
  Orientation orientation;
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct Transform {
  Position translation;
  Quaternion rotation;
};

struct Transforms {
  Transform TBE, TB0, TB1, TB2;
};

} // namespace arm_controller