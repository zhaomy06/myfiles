#pragma once
#include <cstddef>

#include "constraint.h"

namespace ilqr_solver {
class BoundConstraint {
 public:
  BoundConstraint() = default;

  void Init(Constraint *const con, const double ConstrainTolerance,
            const SolverConfig *const config_ptr) {
    solver_config_ptr_ = config_ptr;
    n_ = solver_config_ptr_->ilqr_config().state_size();
    m_ = solver_config_ptr_->ilqr_config().input_size();
    con_ = con;
    ConstrainTolerance_ = ConstrainTolerance;

    index_ = 0;

    // bound constraint size 2
    c_ = VectorXd::Zero(2);
    c_proj_ = VectorXd::Zero(2);
    Irho_ = MatrixXd::Ones(2, 2);

    jac_ = MatrixXd::Zero(2, n_ + m_);
    lambda_ = VectorXd::Zero(2);
    temp_lambda_ = VectorXd::Zero(2);
    dxdu_ = VectorXd::Zero(n_ + m_);
    temp_dxdu_ = VectorXd::Zero(n_ + m_);
    lambda_proj_ = VectorXd::Zero(2);
    proj_jac_ = MatrixXd::Zero(2, 2);
    jac_proj_ = MatrixXd::Zero(2, n_ + m_);
  }

  void UpdateIrho(const VectorXd &c, MatrixXd *I_rho) {
    for (int i = 0; i < c.size(); ++i) {
      if (c(i) < 0 && std::fabs(lambda_(i)) < 1e-3) {
        (*I_rho)(i, i) = 0.0;
      } else {
        (*I_rho)(i, i) = rho_;
      }
    }
  }

  double AugLag(const VectorXd &x, const VectorXd &u, const size_t step) {
    con_->Evaluate(x, u, step, &c_);
    double J = 0.0;
    temp_lambda_ = lambda_ - rho_ * c_;
    NegativeOrthant::Projection(temp_lambda_, &lambda_proj_);
    J = lambda_proj_.squaredNorm() - lambda_.squaredNorm();
    J = J / (2 * rho_);
    return J;
  }

  void AugLagGradientHessian(const VectorXd &x, const VectorXd &u,
                             const size_t step, VectorXd *const dx,
                             VectorXd *const du, MatrixXd *const dxdx,
                             MatrixXd *const dxdu, MatrixXd *const dudu) {
    con_->Evaluate(x, u, step, &c_);
    con_->Jacobian(x, u, step, &jac_);
    NegativeOrthant::Projection(temp_lambda_, &lambda_proj_);
    NegativeOrthant::Jacobian(temp_lambda_, &proj_jac_);

    if (lambda_proj_(0) < 0.0) {
      index_ = 0;
    } else if (lambda_proj_(1) < 0.0) {
      index_ = 1;
    } else {
      return;
    }
    temp_dxdu_.noalias() = proj_jac_(index_, index_) * jac_.row(index_);
    dxdu_.noalias() = -temp_dxdu_ * lambda_proj_(index_);
    (*dx).noalias() += dxdu_.head(n_);
    (*du).noalias() += dxdu_.tail(m_);
    jac_proj_.noalias() = proj_jac_ * jac_;
    (*dxdx).noalias() +=
        rho_ * temp_dxdu_.head(n_) * temp_dxdu_.head(n_).transpose();
    (*dxdu).noalias() +=
        rho_ * temp_dxdu_.head(n_) * temp_dxdu_.tail(m_).transpose();
    (*dudu).noalias() +=
        rho_ * temp_dxdu_.tail(m_) * temp_dxdu_.tail(m_).transpose();
  }

  void UpdateDuals(const VectorXd &x, const VectorXd &u, const size_t step) {
    con_->Evaluate(x, u, step, &c_);
    NegativeOrthant::Projection(lambda_ - rho_ * c_, &lambda_);
  }

  const VectorXd &GetDuals() { return lambda_; }
  const double &GetPenalty() { return rho_; }

  void UpdatePenalties() {
    rho_ *= solver_config_ptr_->cilqr_config().penalty_factor();
  }

  void ResetPenalties() {
    rho_ = solver_config_ptr_->cilqr_config().init_rho();
  }
  void ResetDuals() {
    lambda_.resize(2);
    lambda_.setZero();
  }

  double MaxPenalty() { return rho_; }

  bool CheckViolation() {
    if (MaxViolation() > ConstrainTolerance_) {
      return false;
    }
    return true;
  }

  template <int p = Eigen::Infinity>
  double MaxViolation() {
    double res = 0.0;

    NegativeOrthant::Projection(c_, &c_proj_);
    c_proj_ = c_ - c_proj_;
    res = c_proj_.template lpNorm<p>();

    return res;
  }

  std::uint8_t GetConstraintId() const { return con_->GetConstraintId(); }

 private:
  size_t n_{5};
  size_t m_{1};
  Constraint *con_{nullptr};
  double ConstrainTolerance_{0.0};
  VectorXd c_;
  VectorXd c_proj_;
  double rho_ = 1.0;
  MatrixXd Irho_;
  size_t index_{0};

  MatrixXd jac_;
  VectorXd lambda_;
  VectorXd lambda_proj_;
  VectorXd temp_lambda_;
  VectorXd dxdu_;
  VectorXd temp_dxdu_;

  MatrixXd proj_jac_;
  MatrixXd jac_proj_;
  const SolverConfig *solver_config_ptr_{nullptr};
};

}  // namespace ilqr_solver
