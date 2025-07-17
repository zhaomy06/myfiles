#pragma once

#include "ilqr_define.h"
#include "ilqr_model.h"

namespace ilqr_solver {

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

class NegativeOrthant {
 public:
  NegativeOrthant() = default;
  virtual ~NegativeOrthant() = default;
  static void Projection(const VectorXd &x, VectorXd *const x_proj) {
    for (int i = 0; i < x.size(); ++i) {
      (*x_proj)(i) = std::min(0.0, x(i));
    }
  }
  static void Jacobian(const VectorXd &x, MatrixXd *const jac) {
    for (int i = 0; i < x.size(); ++i) {
      (*jac)(i, i) = x(i) > 0 ? 0 : 1;
    }
  }
  static void Hessian(const VectorXd & /*x*/, const VectorXd & /*b*/,
                      MatrixXd &hess) {
    hess.setZero();
  }
};

class Constraint {
 public:
  Constraint() = default;
  virtual ~Constraint() = default;

  virtual void Evaluate(const VectorXd & /*x*/, const VectorXd & /*u*/,
                        const size_t /*step*/,
                        VectorXd *const /*out*/) const = 0;
  virtual void Jacobian(const VectorXd & /*x*/, const VectorXd & /*u*/,
                        const size_t /*step*/,
                        MatrixXd *const /*jac*/) const = 0;
  virtual std::uint8_t GetConstraintId() const = 0;
};

}  // namespace ilqr_solver
