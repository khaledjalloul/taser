#pragma once

#include "base_controller/types.hpp"

namespace base_controller {

BaseVelocity get_base_velocity(WheelState s, double L, double W);

BasePosition get_base_displacement(BaseVelocity b, double dt,
                                   double theta_current);

} // namespace base_controller