#include "rps_omni_controller/mpc.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using Eigen::MatrixXd;
using Eigen::VectorXd;

OmniMpcController::OmniMpcController(
  double Ts,
  int Np,
  int Nc,
  double a_min,
  double a_max,
  double v_min,
  double v_max)
: Ts_(Ts),
  Np_(Np),
  Nc_(Nc),
  a_min_(a_min),
  a_max_(a_max),
  v_min_(v_min),
  v_max_(v_max),
  last_accel_(Control::Zero()),
  hold_last_control_(true),
  hessian_regularization_(1e-8)
{
  if (Ts_ <= 0.0) {
    throw std::runtime_error("OmniMpcController: Ts must be positive");
  }

  if (Np_ <= 0) {
    Np_ = 1;
  }

  if (Nc_ <= 0) {
    Nc_ = 1;
  }

  if (Nc_ > Np_) {
    Nc_ = Np_;
  }

  if (a_min_ > a_max_) {
    std::swap(a_min_, a_max_);
  }

  if (v_min_ > v_max_) {
    std::swap(v_min_, v_max_);
  }

  updateModelMatrix();

  S_.setZero();
  S_(0, 0) = 30.0;
  S_(1, 1) = 30.0;
  S_(2, 2) = 3.0;
  S_(3, 3) = 3.0;

  Q_.setZero();
  Q_(0, 0) = 5.0;
  Q_(1, 1) = 5.0;
  Q_(2, 2) = 0.5;
  Q_(3, 3) = 0.5;

  R_.setZero();
  R_(0, 0) = 0.05;
  R_(1, 1) = 0.05;

  R_delta_.setZero();
  R_delta_(0, 0) = 2.0;
  R_delta_(1, 1) = 2.0;

  last_solution_ = VectorXd::Zero(2 * Nc_);
}

void OmniMpcController::updateModelMatrix()
{
  A_.setIdentity();

  A_(0, 2) = Ts_;
  A_(1, 3) = Ts_;

  B_.setZero();

  B_(0, 0) = 0.5 * Ts_ * Ts_;
  B_(1, 1) = 0.5 * Ts_ * Ts_;
  B_(2, 0) = Ts_;
  B_(3, 1) = Ts_;
}

void OmniMpcController::initWeights(
  const Eigen::Matrix4d & S,
  const Eigen::Matrix4d & Q,
  const Eigen::Matrix2d & R,
  const Eigen::Matrix2d & R_delta)
{
  S_ = S;
  Q_ = Q;
  R_ = R;
  R_delta_ = R_delta;
}

void OmniMpcController::setHorizon(int Np, int Nc)
{
  if (Np <= 0) {
    Np = 1;
  }

  if (Nc <= 0) {
    Nc = 1;
  }

  if (Nc > Np) {
    Nc = Np;
  }

  Np_ = Np;
  Nc_ = Nc;

  last_solution_ = VectorXd::Zero(2 * Nc_);
}

void OmniMpcController::setAccelerationLimits(double a_min, double a_max)
{
  a_min_ = a_min;
  a_max_ = a_max;

  if (a_min_ > a_max_) {
    std::swap(a_min_, a_max_);
  }
}

void OmniMpcController::setVelocityLimits(double v_min, double v_max)
{
  v_min_ = v_min;
  v_max_ = v_max;

  if (v_min_ > v_max_) {
    std::swap(v_min_, v_max_);
  }
}

void OmniMpcController::setHoldLastControl(bool hold_last_control)
{
  hold_last_control_ = hold_last_control;
}

std::vector<OmniMpcController::State> OmniMpcController::normalizeReference(
  const std::vector<State> & ref_seq) const
{
  std::vector<State> ref_out;
  ref_out.reserve(Np_);

  if (ref_seq.empty()) {
    ref_out.resize(Np_, State::Zero());
    return ref_out;
  }

  for (int i = 0; i < Np_; ++i) {
    if (i < static_cast<int>(ref_seq.size())) {
      ref_out.push_back(ref_seq[i]);
    } else {
      ref_out.push_back(ref_seq.back());
    }
  }

  return ref_out;
}

void OmniMpcController::buildPredictionMatrix(
  MatrixXd & A_bar,
  MatrixXd & B_bar) const
{
  const int nx = 4;
  const int nu = 2;

  A_bar.setZero(nx * Np_, nx);
  B_bar.setZero(nx * Np_, nu * Nc_);

  std::vector<Eigen::Matrix4d> A_power(Np_ + 1);
  A_power[0].setIdentity();

  for (int i = 1; i <= Np_; ++i) {
    A_power[i] = A_power[i - 1] * A_;
  }

  for (int i = 0; i < Np_; ++i) {

    A_bar.block<nx, nx>(nx * i, 0) = A_power[i + 1];

    for (int j = 0; j < Nc_; ++j) {
      Eigen::Matrix<double, 4, 2> block;
      block.setZero();

      if (!hold_last_control_ || j < Nc_ - 1) {
        if (j <= i) {

          block = A_power[i - j] * B_;
        }
      } else {

        if (i >= j) {
          for (int s = 0; s <= i - j; ++s) {
            block += A_power[s] * B_;
          }
        }
      }

      B_bar.block<nx, nu>(nx * i, nu * j) = block;
    }
  }
}

void OmniMpcController::buildQPMatrix(
  const State & x0,
  const std::vector<State> & ref_seq,
  MatrixXd & H,
  VectorXd & f) const
{
  const int nx = 4;
  const int nu = 2;

  const int x_dim = nx * Np_;
  const int u_dim = nu * Nc_;

  const auto ref = normalizeReference(ref_seq);

  MatrixXd A_bar;
  MatrixXd B_bar;
  buildPredictionMatrix(A_bar, B_bar);

  VectorXd X_ref(x_dim);
  X_ref.setZero();

  for (int i = 0; i < Np_; ++i) {
    X_ref.segment<nx>(nx * i) = ref[i];
  }

  MatrixXd Q_bar(x_dim, x_dim);
  Q_bar.setZero();

  for (int i = 0; i < Np_; ++i) {
    Q_bar.block<nx, nx>(nx * i, nx * i) = Q_;
  }

  MatrixXd R_bar(u_dim, u_dim);
  R_bar.setZero();

  for (int i = 0; i < Nc_; ++i) {
    R_bar.block<nu, nu>(nu * i, nu * i) = R_;
  }

  const int terminal_row = nx * (Np_ - 1);

  MatrixXd A_N = A_bar.block(terminal_row, 0, nx, nx);
  MatrixXd B_N = B_bar.block(terminal_row, 0, nx, u_dim);

  State x_ref_N = ref[Np_ - 1];

  VectorXd tracking_error_const = A_bar * x0 - X_ref;
  VectorXd terminal_error_const = A_N * x0 - x_ref_N;

  H = 2.0 * (
    B_bar.transpose() * Q_bar * B_bar +
    B_N.transpose() * S_ * B_N +
    R_bar);

  f = 2.0 * (
    B_bar.transpose() * Q_bar * tracking_error_const +
    B_N.transpose() * S_ * terminal_error_const);

  if (R_delta_.norm() > 1e-12) {
    MatrixXd D(u_dim, u_dim);
    D.setZero();

    VectorXd d(u_dim);
    d.setZero();

    for (int i = 0; i < Nc_; ++i) {
      const int row = nu * i;
      const int col = nu * i;

      D.block<nu, nu>(row, col) = Eigen::Matrix2d::Identity();

      if (i == 0) {
        d.segment<nu>(row) = last_accel_;
      } else {
        const int col_prev = nu * (i - 1);
        D.block<nu, nu>(row, col_prev) = -Eigen::Matrix2d::Identity();
      }
    }

    MatrixXd R_delta_bar(u_dim, u_dim);
    R_delta_bar.setZero();

    for (int i = 0; i < Nc_; ++i) {
      R_delta_bar.block<nu, nu>(nu * i, nu * i) = R_delta_;
    }

    H += 2.0 * D.transpose() * R_delta_bar * D;
    f += -2.0 * D.transpose() * R_delta_bar * d;
  }

  H = 0.5 * (H + H.transpose());

  H.diagonal().array() += hessian_regularization_;
}
void OmniMpcController::buildConstraintMatrix(
  const State & x0,
  MatrixXd & A_qp,
  VectorXd & lower_bound,
  VectorXd & upper_bound) const
{
  const int nx = 4;
  const int nu = 2;

  const int u_dim = nu * Nc_;

  MatrixXd A_bar;
  MatrixXd B_bar;
  buildPredictionMatrix(A_bar, B_bar);

  MatrixXd A_acc = MatrixXd::Identity(u_dim, u_dim);
  VectorXd l_acc = VectorXd::Constant(u_dim, a_min_);
  VectorXd u_acc = VectorXd::Constant(u_dim, a_max_);

  MatrixXd C_v = MatrixXd::Zero(nu * Np_, nx * Np_);

  for (int i = 0; i < Np_; ++i) {
    C_v(nu * i + 0, nx * i + 2) = 1.0;
    C_v(nu * i + 1, nx * i + 3) = 1.0;
  }

  MatrixXd A_vel = C_v * B_bar;
  VectorXd v_const = C_v * A_bar * x0;

  VectorXd l_vel = VectorXd::Constant(nu * Np_, v_min_) - v_const;
  VectorXd u_vel = VectorXd::Constant(nu * Np_, v_max_) - v_const;

  const int n_constraints = A_acc.rows() + A_vel.rows();

  A_qp.resize(n_constraints, u_dim);
  lower_bound.resize(n_constraints);
  upper_bound.resize(n_constraints);

  int row = 0;

  A_qp.block(row, 0, A_acc.rows(), u_dim) = A_acc;
  lower_bound.segment(row, A_acc.rows()) = l_acc;
  upper_bound.segment(row, A_acc.rows()) = u_acc;
  row += A_acc.rows();

  A_qp.block(row, 0, A_vel.rows(), u_dim) = A_vel;
  lower_bound.segment(row, A_vel.rows()) = l_vel;
  upper_bound.segment(row, A_vel.rows()) = u_vel;
}

bool OmniMpcController::solveConstrainedQP(
  const MatrixXd & H,
  const VectorXd & f,
  const MatrixXd & A_qp,
  const VectorXd & lower_bound,
  const VectorXd & upper_bound,
  VectorXd & U) const
{
  const int n = static_cast<int>(H.cols());
  const int m = static_cast<int>(A_qp.rows());

  if (n <= 0 || H.rows() != n || f.size() != n || A_qp.cols() != n ||
    lower_bound.size() != m || upper_bound.size() != m)
  {
    return false;
  }

  const double rho = 10.0;
  const double sigma = 1e-6;
  const int max_iter = 80;
  const double abs_tol = 1e-4;
  const double rel_tol = 1e-4;

  MatrixXd K = H;
  K += rho * A_qp.transpose() * A_qp;
  K.diagonal().array() += sigma;
  K = 0.5 * (K + K.transpose());

  Eigen::LLT<MatrixXd> llt(K);
  if (llt.info() != Eigen::Success) {
    return false;
  }

  if (last_solution_.size() == n) {
    U = last_solution_;
  } else {
    U = VectorXd::Zero(n);
  }

  VectorXd z = A_qp * U;
  VectorXd y = VectorXd::Zero(m);

  for (int i = 0; i < m; ++i) {
    z(i) = std::clamp(z(i), lower_bound(i), upper_bound(i));
  }

  bool converged = false;

  for (int iter = 0; iter < max_iter; ++iter) {
    const VectorXd rhs = -f + rho * A_qp.transpose() * (z - y);
    U = llt.solve(rhs);

    if (llt.info() != Eigen::Success || !U.allFinite()) {
      return false;
    }

    const VectorXd A_U = A_qp * U;
    const VectorXd z_old = z;

    z = A_U + y;

    for (int i = 0; i < m; ++i) {
      z(i) = std::clamp(z(i), lower_bound(i), upper_bound(i));
    }

    y += A_U - z;

    const double primal_res = (A_U - z).norm();
    const double dual_res = (rho * A_qp.transpose() * (z - z_old)).norm();

    const double eps_primal = std::sqrt(static_cast<double>(m)) * abs_tol +
      rel_tol * std::max(A_U.norm(), z.norm());

    const double eps_dual = std::sqrt(static_cast<double>(n)) * abs_tol +
      rel_tol * (rho * A_qp.transpose() * y).norm();

    if (primal_res <= eps_primal && dual_res <= eps_dual) {
      converged = true;
      break;
    }
  }

  if (!U.allFinite()) {
    return false;
  }

  const VectorXd A_U = A_qp * U;
  double max_violation = 0.0;

  for (int i = 0; i < m; ++i) {
    if (A_U(i) < lower_bound(i)) {
      max_violation = std::max(max_violation, lower_bound(i) - A_U(i));
    } else if (A_U(i) > upper_bound(i)) {
      max_violation = std::max(max_violation, A_U(i) - upper_bound(i));
    }
  }

  if (!converged && max_violation > 5e-2) {
    return false;
  }

  return true;
}
OmniMpcController::Control OmniMpcController::clipAcceleration(
  const Control & u) const
{
  Control out;

  for (int i = 0; i < 2; ++i) {
    if (!std::isfinite(u(i))) {
      out(i) = 0.0;
    } else {
      out(i) = std::clamp(u(i), a_min_, a_max_);
    }
  }

  return out;
}

Eigen::Vector2d OmniMpcController::clipVelocity(
  const Eigen::Vector2d & v) const
{
  Eigen::Vector2d out;

  for (int i = 0; i < 2; ++i) {
    if (!std::isfinite(v(i))) {
      out(i) = 0.0;
    } else {
      out(i) = std::clamp(v(i), v_min_, v_max_);
    }
  }

  return out;
}

OmniMpcController::Control OmniMpcController::solveAcceleration(
  const State & x0,
  const std::vector<State> & ref_seq)
{
  if (ref_seq.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("OmniMpcController"),
      "Reference sequence is empty, return zero acceleration");
    last_accel_.setZero();
    return Control::Zero();
  }

  const int u_dim = 2 * Nc_;

  MatrixXd H(u_dim, u_dim);
  VectorXd f(u_dim);

  buildQPMatrix(x0, ref_seq, H, f);

  MatrixXd A_qp;
  VectorXd lower_bound;
  VectorXd upper_bound;

  buildConstraintMatrix(x0, A_qp, lower_bound, upper_bound);

  VectorXd U(u_dim);

  if (!solveConstrainedQP(H, f, A_qp, lower_bound, upper_bound, U) || U.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("OmniMpcController"),
      "Constrained QP solve failed, fallback to unconstrained solution with clipping");

    Eigen::LLT<MatrixXd> llt(H);

    if (llt.info() != Eigen::Success) {
      last_accel_.setZero();
      return Control::Zero();
    }

    U = llt.solve(-f);

    if (llt.info() != Eigen::Success || U.size() < 2 || !U.allFinite()) {
      last_accel_.setZero();
      return Control::Zero();
    }
  }

  Control accel_cmd = U.head<2>();

  accel_cmd = clipAcceleration(accel_cmd);

  if (U.size() >= 2) {
    U.head<2>() = accel_cmd;
  }

  last_accel_ = accel_cmd;
  last_solution_ = U;

  return accel_cmd;
}

Eigen::Vector2d OmniMpcController::solveVelocityCommand(
  const State & x0,
  const std::vector<State> & ref_seq)
{
  const Control accel_cmd = solveAcceleration(x0, ref_seq);

  const Eigen::Vector2d v_now = x0.segment<2>(2);
  Eigen::Vector2d v_cmd = v_now + Ts_ * accel_cmd;

  v_cmd = clipVelocity(v_cmd);

  last_accel_ = (v_cmd - v_now) / Ts_;

  return v_cmd;
}

void OmniMpcController::reset()
{
  last_accel_.setZero();
  last_solution_ = VectorXd::Zero(2 * Nc_);
}
