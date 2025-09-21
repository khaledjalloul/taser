#pragma once

#include <optional>

#include "taser_cpp/types.hpp"

namespace taser_cpp::locomotion {

/**
 * Kinematics class for the robot's differential drive base
 */
class Kinematics {
public:
  /**
   * Constructor
   * @param L Distance between the wheels
   * @param wheel_radius Radius of the wheels
   * @param dt Time step
   */
  Kinematics(double L, double wheel_radius, double dt);

  /**
   * Set the base velocity which internally updates the wheel velocities
   * @param v Linear velocity
   * @param omega Angular velocity
   */
  void set_base_velocity(double v, double omega);

  /**
   * Set the wheel velocities which internally updates the base velocity
   * @param v_l Left wheel velocity
   * @param v_r Right wheel velocity
   */
  void set_wheel_velocities(double v_l, double v_r);

  /**
   * Update the robot's pose one step forward based on the wheel velocities
   * @note Does not physically move the robot, only updates the internal pose
   * @param dt (Optional) Time step, defaults to the one given in constructor
   */
  void step(std::optional<double> dt = std::nullopt);

  Pose2D pose{0, 0, 0};
  BaseVelocity base_velocity; // Base velocity
  double v_l, v_r;            // Wheel velocities

private:
  double L_, wheel_radius_;
  double dt_;
};

} // namespace taser_cpp::locomotion