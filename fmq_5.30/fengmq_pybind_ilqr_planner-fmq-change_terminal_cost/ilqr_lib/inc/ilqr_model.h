#pragma once

#include "ilqr_define.h"

// state of model
using State = Eigen::VectorXd;

// control of model
using Control = Eigen::VectorXd;

// definition of ilqr matrix for each iteration
using XkMT = Eigen::VectorXd;
using UkMT = Eigen::VectorXd;
using LxMT = Eigen::VectorXd;  // lx size is equal to xk
using LuMT = Eigen::VectorXd;  // lu size is equal to uk
using LxxMT = Eigen::MatrixXd;
using LxuMT = Eigen::MatrixXd;
using LuuMT = Eigen::MatrixXd;
using FxMT = Eigen::MatrixXd;
using FuMT = Eigen::MatrixXd;
// NOLINTNEXTLINE(readability-identifier-naming)
using kMT = Eigen::VectorXd;
using KMT = Eigen::MatrixXd;

// vector of definition of ilqr matrix for all iteration
using StateVec = std::vector<State>;
using ControlVec = std::vector<Control>;
using LxMTVec = std::vector<LxMT>;
using LuMTVec = std::vector<LuMT>;
using LxxMTVec = std::vector<LxxMT>;
using LxuMTVec = std::vector<LxuMT>;
using LuuMTVec = std::vector<LuuMT>;
using FxMTVec = std::vector<FxMT>;
using FuMTVec = std::vector<FuMT>;

// NOLINTNEXTLINE(readability-identifier-naming)
using kMTVec = std::vector<kMT>;
using KMTVec = std::vector<KMT>;

namespace ilqr_solver {
class ILqrModel {
 public:
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ILqrModel() {}

  virtual ~ILqrModel() = default;

  virtual State UpdateDynamicsOneStep(const State &x, const Control &u,
                                      const size_t &step) const = 0;

  virtual void GetDynamicsDerivatives(const State &x, const Control & /*u*/,
                                      const size_t &step, FxMT *const f_x,
                                      FuMT *const f_u) const = 0;
  virtual void LimitControl(const State &x, Control *const u) const = 0;
};

}  // namespace ilqr_solver
