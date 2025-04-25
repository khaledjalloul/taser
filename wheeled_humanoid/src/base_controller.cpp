#include "wheeled_humanoid/base_controller.hpp"

namespace wheeled_humanoid {

BaseController::BaseController(double dt, int N, double v_max, double omega_max)
    : dt_(dt), N_(N), v_max_(v_max), omega_max_(omega_max) {
  nx_ = 3; // x, y, theta
  nu_ = 2; // v, omega

  Q_ = Matrix::Identity(2, 2) * 10;
  R_ = Matrix::Identity(nu_, nu_);
  R_(0, 0) = 0.5;
  R_(1, 1) = 0.1;

  num_vars_ = nx_ * (N_ + 1) + nu_ * N;
  num_constraints_ = nx_ * (N_ + 1) + nx_ + N * nu_;

  solver_.settings()->setVerbosity(false);
  solver_.data()->setNumberOfVariables(num_vars_);
  solver_.data()->setNumberOfConstraints(num_constraints_);
}

BaseVelocity BaseController::step(const Pose2D &x0, const Path &desired_path) {
  Matrix A, B;
  get_linearized_model(x0, A, B);

  // std::vector<Eigen::Vector2d> local_ref(inter_path.begin() + step,
  //                                        inter_path.begin() + step + N_ + 1);

  set_up_QP(x0, desired_path, A, B);

  auto status = solver_.solveProblem();
  if (status != OsqpEigen::ErrorExitFlag::NoError) {
    std::cerr << "Failed to solve!";
    return BaseVelocity{0.0, 0.0};
  }

  Vector sol = solver_.getSolution();
  auto u0 = sol.segment((N_ + 1) * nx_, nu_);

  return BaseVelocity{u0(0), u0(1)};
}

void BaseController::get_linearized_model(const Pose2D &x0, Matrix &A,
                                          Matrix &B) const {
  A = Matrix::Identity(3, 3);
  A(0, 2) = -sin(x0.theta) * dt_;
  A(1, 2) = cos(x0.theta) * dt_;

  B = Matrix::Zero(3, 2);
  B(0, 0) = dt_ * cos(x0.theta);
  B(1, 0) = dt_ * sin(x0.theta);
  B(2, 1) = dt_;
}

void BaseController::set_up_QP(const Pose2D &x0, const Path &desired_path,
                               const Matrix &A, const Matrix &B) {
  solver_.clearSolver();
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();

  Matrix H = Matrix::Zero(num_vars_, num_vars_);
  Vector g = Vector::Zero(num_vars_);
  Matrix Aeq = Matrix::Zero(num_constraints_, num_vars_);
  Vector lb = Vector::Zero(num_constraints_);
  Vector ub = Vector::Zero(num_constraints_);

  for (int i = 0; i < N_; i++) {
    int xi = i * nx_;
    int ui = nx_ * (N_ + 1) + i * nu_;

    H.block(xi, xi, 2, 2) = Q_;
    H.block(ui, ui, nu_, nu_) = R_;

    g.segment(xi, 2) = -Q_ * desired_path[i];

    Aeq.block((i + 1) * nx_, xi, nx_, nx_) = -A;
    Aeq.block((i + 1) * nx_, ui, nx_, nu_) = -B;
    Aeq.block((i + 1) * nx_, (i + 1) * nx_, nx_, nx_) =
        Matrix::Identity(nx_, nx_);

    Aeq.block((N_ + 2) * nx_ + i * nu_, (N_ + 1) * nx_ + i * nu_, nu_, nu_) =
        Matrix::Identity(nu_, nu_);
    lb.segment((N_ + 2) * nx_ + i * nu_, nu_) << -v_max_, -omega_max_;
    ub.segment((N_ + 2) * nx_ + i * nu_, nu_) << v_max_, omega_max_;
  }

  Aeq.block(0, 0, nx_, nx_) = Matrix::Identity(nx_, nx_);
  lb.segment(0, nx_) << x0.x, x0.y, x0.theta;
  ub.segment(0, nx_) << x0.x, x0.y, x0.theta;

  Aeq.block((N_ + 1) * nx_, N_ * nx_, 2, 2) = Matrix::Identity(2, 2);
  lb.segment((N_ + 1) * nx_, 2) << desired_path[N_].x(), desired_path[N_].y();
  ub.segment((N_ + 1) * nx_, 2) << desired_path[N_].x(), desired_path[N_].y();

  solver_.data()->setHessianMatrix((Eigen::SparseMatrix<double>)H.sparseView());
  solver_.data()->setGradient(g);
  solver_.data()->setLinearConstraintsMatrix(
      (Eigen::SparseMatrix<double>)Aeq.sparseView());
  solver_.data()->setBounds(lb, ub);
  solver_.initSolver();
}

} // namespace wheeled_humanoid