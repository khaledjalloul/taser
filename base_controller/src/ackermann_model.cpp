#include "base_controller/ackermann_model.hpp"

namespace base_controller {

BaseVelocity get_base_velocity(WheelState s, double L, double W) {
  auto v = (s.v_l + s.v_r) / 2;

  auto w1 = (s.v_r - s.v_l) / W;
  auto w2 = v * tan(s.steering) / L;

  return {v, w1 + w2};
};

BasePosition get_base_displacement(BaseVelocity b, double dt,
                                   double theta_current) {
  if (b.w == 0) {
    auto x_delta = b.v * cos(theta_current) * dt;
    auto y_delta = b.v * sin(theta_current) * dt;

    return {x_delta, y_delta, 0};
  }

  auto x_delta =
      (b.v / b.w) * (sin(b.w * dt + theta_current) - sin(theta_current));
  auto y_delta =
      (b.v / b.w) * (-cos(b.w * dt + theta_current) + cos(theta_current));
  auto theta_delta = b.w * dt;

  return {x_delta, y_delta, theta_delta};
};

} // namespace base_controller