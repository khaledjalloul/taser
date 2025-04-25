#pragma once

#include <eigen3/Eigen/Dense>

namespace wheeled_humanoid {

using Vector3 = Eigen::Vector3d;
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

using Axis = Vector3;
using Quaternion = Eigen::Quaterniond;
using Jacobian = Eigen::Matrix<double, 6, 3>;

using ArmJointPositions = Vector3;
using ArmJointVelocities = Vector3;
using RobotJointPositions = Eigen::Vector<double, 10>;
using RobotJointVelocities = Eigen::Vector<double, 10>;

struct Pose2D {
  double x, y, theta;
};

struct Position3D {
  double x, y, z;
};

struct Orientation3D {
  double roll, pitch, yaw;
};

struct Pose3D {
  Position3D position;
  Orientation3D orientation;
};

struct Twist3D {
  Position3D linear;
  Orientation3D angular;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct Transforms {
  Transform TBE, TB0, TB1, TB2;
};

struct BaseVelocity {
  double v, omega;
};

} // namespace wheeled_humanoid