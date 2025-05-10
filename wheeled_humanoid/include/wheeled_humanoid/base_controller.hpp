#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

class BaseController {
public:
  BaseController(double dt, int N = 10, double v_max = 1.0,
                 double omega_max = 2.0);

  BaseVelocity step(const Pose2D &x0, const Path &x_ref,
                    const VelocityProfile &u_ref);

  void get_linearized_model(const Pose2D &x0, const BaseVelocity &u0, Matrix &A,
                            Matrix &B) const;

  int N;

private:
  void set_up_QP_(const Pose2D &x0, const Path &x_ref,
                  const VelocityProfile &u_ref);

  double dt_, v_max_, omega_max_;
  int nx_, nu_, num_vars_, num_constraints_;
  Matrix Q_, R_;
  OsqpEigen::Solver solver_;
};

} // namespace wheeled_humanoid