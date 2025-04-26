#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

class BaseController {
public:
  BaseController(double dt, int N = 10, double v_max = 1.0,
                 double omega_max = 2.0);

  BaseVelocity step(const Pose2D &x0, const Path &desired_path);

  int N;

private:
  void get_linearized_model(const Pose2D &x0, Matrix &A, Matrix &B) const;

  void set_up_QP(const Pose2D &x0, const Path &desired_path, const Matrix &A,
                 const Matrix &B);

  double dt_, v_max_, omega_max_;
  int nx_, nu_, num_vars_, num_constraints_;
  Matrix Q_, R_;
  OsqpEigen::Solver solver_;
};

} // namespace wheeled_humanoid