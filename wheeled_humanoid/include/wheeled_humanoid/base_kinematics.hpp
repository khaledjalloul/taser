#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

// Ackermann model
class BaseKinematics {
public:
  BaseKinematics(double L, double W, double wheel_radius);

  void set_wheel_state(const WheelState &s);

  void set_properties(double L, double W);

  BaseVelocity calculate_base_velocity() const;

  Pose2D get_base_displacement(double dt) const;

  Pose2D p{0, 0, 0};
  BaseVelocity v{0, 0};

private:
  double L_, W_, wheel_radius_ = 0.5;
  WheelState s_{0, 0, 0};
};

} // namespace wheeled_humanoid