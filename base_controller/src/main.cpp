#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "base_controller/ackermann_model.hpp"

using namespace base_controller;

int main(int argc, char **argv) {
  auto s = WheelState{1, 1, 0};
  auto L = 1.0;
  auto W = 0.5;

  auto v = get_base_velocity(s, L, W);
  auto p = get_base_displacement(v, 0.1, 0.1);

  std::cout << "v: " << v.v << ", w: " << v.w << std::endl;
  std::cout << "x: " << p.x << ", y: " << p.y << ", theta: " << p.theta
            << std::endl;

  return 0;
}