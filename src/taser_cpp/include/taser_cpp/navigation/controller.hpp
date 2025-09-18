#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include "taser_cpp/types.hpp"

namespace taser_cpp::navigation {

/**
 * Tracking controller for the base using Model Predictive Control
 */
class Controller {
public:
  /**
   * Constructor
   * @param dt Time step
   * @param N Number of prediction steps
   * @param v_max Maximum linear velocity
   * @param omega_max Maximum angular velocity
   */
  Controller(double dt, int N, double v_max = 1.0, double omega_max = 2.0);

  /**
   * Compute the desired base velocity command
   * @param x0 Current pose of the base
   * @param x_ref Reference path
   * @param u_ref Reference velocity profile
   * @return Base velocity command
   */
  BaseVelocity step(const Pose2D &x0, const Path &x_ref,
                    const VelocityProfile &u_ref);

  /**
   * Get the linearized kinematics model of the base (Differential drive)
   * @param[in] x0 Current pose of the base
   * @param[in] u0 Current base velocity
   * @param[out] A State transition matrix
   * @param[out] B Control input matrix
   */
  void get_linearized_model(const Pose2D &x0, const BaseVelocity &u0, Matrix &A,
                            Matrix &B) const;

  int N;

private:
  /**
   * Set up the QP problem for the MPC controller
   * @param x0 Current pose of the base
   * @param x_ref Reference path
   * @param u_ref Reference velocity profile
   */
  void set_up_QP_(const Pose2D &x0, const Path &x_ref,
                  const VelocityProfile &u_ref);

  double dt_, v_max_, omega_max_;
  int nx_, nu_, num_vars_, num_constraints_;
  Matrix Q_, R_;
  OsqpEigen::Solver solver_;
};

} // namespace taser_cpp::navigation