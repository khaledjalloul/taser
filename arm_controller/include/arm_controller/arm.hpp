#pragma once

#include <eigen3/Eigen/Dense>
#include <string>

#include "arm_controller/transform_listener.hpp"

namespace arm_controller {

using Vector = Eigen::Vector3d;
using Jacobian = Eigen::Matrix<double, 6, 3>;

class Arm {

public:
  Arm(std::string name, std::string inertial_frame,
      std::shared_ptr<TransformListener> transformListener);

  Jacobian get_geometric_jacobian();

private:
  std::string name_, inertial_frame_;
  std::shared_ptr<TransformListener> transformListener_;
  Vector n0_{0, 1, 0};
  Vector n1_{0, 0, 1};
  Vector n2_{1, 0, 0};
};

} // namespace arm_controller