#pragma once

#include <optional>

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

// Differential drive model
class BaseKinematics {
public:
  BaseKinematics(double L, double wheel_radius, double dt);

  void set_L(double L) { L_ = L; }

  void set_base_velocity(double v, double omega);

  void set_wheel_velocities(double v_l, double v_r);

  void step(std::optional<double> dt = std::nullopt);

  Pose2D pose{0, 0, 0};
  BaseVelocity base_velocity; // Base velocity
  double v_l, v_r;            // Wheel velocities

private:
  double L_, wheel_radius_;
  double dt_;
};

} // namespace wheeled_humanoid