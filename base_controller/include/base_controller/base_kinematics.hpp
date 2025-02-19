#pragma once

#include <optional>

#include "base_controller/types.hpp"

namespace base_controller {

// Ackermann model
class BaseKinematics {
public:
  BaseKinematics(double L, double W, double wheel_radius);

  void set_wheel_state(const WheelState &s);

  BaseVelocity calculate_base_velocity() const;

  BasePosition get_base_displacement(double dt) const;

  BasePosition p;
  BaseVelocity v;

private:
  double L_, W_, wheel_radius_;
  WheelState s_;
};

} // namespace base_controller