#include "wheeled_humanoid/base_controller.hpp"

namespace wheeled_humanoid {

BaseController::BaseController(double dt, int N, double v_max, double omega_max)
    : dt_(dt), N(N), v_max_(v_max), omega_max_(omega_max) {
  nx_ = 3; // x, y, theta
  nu_ = 2; // v, omega

  Q_ = Matrix::Identity(nx_, nx_) * 10;
  R_ = Matrix::Identity(nu_, nu_);
  R_(0, 0) = 0.5;
  R_(1, 1) = 0.1;

  num_vars_ = nx_ * (N + 1) + nu_ * N;
  num_constraints_ = nx_ * (N + 1) + nx_ + N * nu_;

  solver_.settings()->setVerbosity(false);
  solver_.data()->setNumberOfVariables(num_vars_);
  solver_.data()->setNumberOfConstraints(num_constraints_);
}

BaseVelocity BaseController::step(const Pose2D &x0, const Path &x_ref,
                                  const VelocityProfile &u_ref) {
  if (x_ref.size() < N) {
    std::cerr << "Cannot step base MPC controller, reference path x_ref has "
                 "less than N elements."
              << std::endl;
    return BaseVelocity();
  }
  if (u_ref.size() < N) {
    std::cerr << "Cannot step base MPC controller, reference velocity profile "
                 "u_ref has less than N elements."
              << std::endl;
    return BaseVelocity();
  }

  set_up_QP_(x0, x_ref, u_ref);

  auto status = solver_.solveProblem();
  if (solver_.getStatus() != OsqpEigen::Status::Solved) {
    std::cerr << "Failed to solve MPC problem!" << std::endl;
    return BaseVelocity{0.0, 0.0};
  }

  Vector sol = solver_.getSolution();
  auto delta_u0 = sol.segment((N + 1) * nx_, nu_);

  BaseVelocity u0{u_ref[0].v + delta_u0(0), u_ref[0].omega + delta_u0(1)};
  return u0;
}

void BaseController::get_linearized_model(const Pose2D &x0,
                                          const BaseVelocity &u0, Matrix &A,
                                          Matrix &B) const {
  /**
   * Linearization:
   * - x_dot = f(x,u) = f(x0,u0) + df/dx(x0,u0) * (x-x0) + df/du(x0,u0) * (u-u0)
   * - delta_x_dot = A * delta_x + B * delta_u
   */

  A = Matrix::Identity(nx_, nx_);
  B = Matrix::Zero(nx_, nu_);

  // x_dot = v * cos(theta)
  A(0, 2) = -sin(x0.theta) * u0.v * dt_;
  B(0, 0) = cos(x0.theta) * dt_;

  // y_dot = v * sin(theta)
  A(1, 2) = cos(x0.theta) * u0.v * dt_;
  B(1, 0) = sin(x0.theta) * dt_;

  // theta_dot = omega
  B(2, 1) = dt_;
}

void BaseController::set_up_QP_(const Pose2D &x0, const Path &x_ref,
                                const VelocityProfile &u_ref) {
  solver_.clearSolver();
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();

  Matrix H = Matrix::Zero(num_vars_, num_vars_);
  Vector g = Vector::Zero(num_vars_);
  Matrix A_constr = Matrix::Zero(num_constraints_, num_vars_);
  Vector lb = Vector::Zero(num_constraints_);
  Vector ub = Vector::Zero(num_constraints_);

  for (int i = 0; i < N; i++) {
    Matrix A, B;
    get_linearized_model(x_ref[0], u_ref[0], A, B);

    int xi = i * nx_;
    int ui = nx_ * (N + 1) + i * nu_;

    /**
     * Cost (For N = 2):
     * Desired: delta_x[k]^T * Q * delta_x[k] + delta_u[k]^T * R * delta_u[k]
     * Osqp: 0.5 * x^T * H * x + g^T * x
     * H: Q 0 0 0 0 -- delta_x0 (redundant)
     *    0 Q 0 0 0 -- delta_x1
     *    0 0 0 0 0 -- delta_x2 (not needed, terminal constraint)
     *    0 0 0 R 0 -- delta_u0
     *    0 0 0 0 R -- delta_u1
     * g: 0 -- delta_x0
     *    0 -- delta_x1
     *    0 -- delta_x2
     *    0 -- delta_u0
     *    0 -- delta_u1
     */

    H.block(xi, xi, nx_, nx_) = Q_;
    H.block(ui, ui, nu_, nu_) = R_;

    /**
     * Constraints (For N = 2):
     * Dynamics: delta_x[k+1] = A * delta_x[k] + B * delta_u[k]
     * Terminal constraint: delta_x[N] = 0
     * Osqp: l <= Ax <= u
     * A: I  0  0  0  0  -- delta_x0 = x(0) - x_ref[0]
     *    -A I  0  -B 0  -- delta_x1 = A * delta_x0 + B * delta_u0
     *    0  -A I  0  -B -- delta_x2 = A * delta_x1 + B * delta_u1
     *    0  0  I  0  0  -- delta_x2 = 0
     *    0  0  0  I  0  -- delta_u0 + u_ref[0] <= max_vel (TODO)
     *    0  0  0  0  I  -- delta_u1 + u_ref[1] <= max_vel (TODO)
     * lb: x(0) - x_ref[0]  -- delta_x0
     *     0                -- delta_x1
     *     0                -- delta_x2
     *     0                -- delta_x2
     *     v_min - u_ref[0] -- delta_u0
     *     v_min - u_ref[1] -- delta_u0
     * ub: x(0) - x_ref[0]  -- delta_x0
     *     0                -- delta_x1
     *     0                -- delta_x2
     *     0                -- delta_x2
     *     v_max - u_ref[0] -- delta_u0
     *     v_max - u_ref[1] -- delta_u0
     */

    A_constr.block((i + 1) * nx_, xi, nx_, nx_) = -A;
    A_constr.block((i + 1) * nx_, ui, nx_, nu_) = -B;
    A_constr.block((i + 1) * nx_, (i + 1) * nx_, nx_, nx_) =
        Matrix::Identity(nx_, nx_);

    // TODO: Fix velocity constraints, probably needs Dubins path to work
    // A_constr.block((N + 2) * nx_ + i * nu_, (N + 1) * nx_ + i * nu_, nu_,
    // nu_) = Matrix::Identity(nu_, nu_); lb.segment((N + 2) * nx_ + i * nu_,
    // nu_) << -v_max_, -omega_max_; ub.segment((N + 2) * nx_ + i * nu_, nu_) <<
    // v_max_, omega_max_;
  }

  // Initial constraint
  A_constr.block(0, 0, nx_, nx_) = Matrix::Identity(nx_, nx_);
  lb.segment(0, nx_) << x0.x - x_ref[0].x, x0.y - x_ref[0].y,
      x0.theta - x_ref[0].theta;
  ub.segment(0, nx_) << x0.x - x_ref[0].x, x0.y - x_ref[0].y,
      x0.theta - x_ref[0].theta;

  // Terminal constraint
  A_constr.block((N + 1) * nx_, N * nx_, nx_, nx_) = Matrix::Identity(nx_, nx_);

  solver_.data()->setHessianMatrix((Eigen::SparseMatrix<double>)H.sparseView());
  solver_.data()->setGradient(g);
  solver_.data()->setLinearConstraintsMatrix(
      (Eigen::SparseMatrix<double>)A_constr.sparseView());
  solver_.data()->setBounds(lb, ub);
  solver_.initSolver();
}

} // namespace wheeled_humanoid