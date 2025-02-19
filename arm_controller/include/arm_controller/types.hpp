#pragma once

#include <eigen3/Eigen/Dense>

namespace arm_controller {

using Vector3 = Eigen::Vector3d;
using Matrix = Eigen::MatrixXd;

using Axis = Vector3;
using Quaternion = Eigen::Quaterniond;
using Jacobian = Eigen::Matrix<double, 6, 3>;

using ArmJointState = Vector3;

struct Position {
  double x, y, z;
};

struct Orientation {
  double roll, pitch, yaw;
};

struct Pose {
  Position position;
  Orientation orientation;
};

struct Twist {
  Position linear;
  Orientation angular;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct Transforms {
  Transform TBE, TB0, TB1, TB2;
};

} // namespace arm_controller