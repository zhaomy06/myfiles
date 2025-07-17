#pragma once

#include <cstddef>

#include "bound_constraint.h"
#include "constraint.h"
#include "ilqr_define.h"
#include "ilqr_model.h"

namespace ilqr_solver {
class BaseCostTerm {
 public:
  virtual ~BaseCostTerm() = default;
  virtual double GetCost(const State & /*x*/, const Control & /*u*/,
                         const size_t /*step*/) const {
    return 0.0;
  }
  virtual void ComputeCommonTerms(const State & /*x*/, const Control & /*u*/,
                                  const size_t /*step*/) {}
  virtual void GetGradientHessian(const State & /*x*/, const Control & /*u*/,
                                  const size_t /*step*/, LxMT *const /*lx*/,
                                  LuMT *const /*lu*/, LxxMT *const /*lxx*/,
                                  LxuMT *const /*lxu*/,
                                  LuuMT *const /*luu*/) const {}
  virtual uint8_t GetCostId() const { return 0; }
};

class CostManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CostManager(const ILqrModel *const model_ptr,
              const SolverConfig *const solver_config_ptr,
              BaseCostTerm *const common_term) {
    ilqr_model_ptr_ = model_ptr;
    solver_config_ptr_ = solver_config_ptr;
    common_term_ = common_term;
  }
  virtual ~CostManager() = default;

 public:
  void InitGuess(StateVec *const x0, ControlVec *const u0,
                 double *const init_cost, const bool &update_init_seed,
                 int *init_seed_index);

  double GetCost(const State &x, const Control &u, const size_t &step);
  double GetTerminalCost(const State &x);

  void GetGradientHessian(const State &x, const Control &u, const size_t &step,
                          LxMT *const lx, LuMT *const lu, LxxMT *const lxx,
                          LxuMT *const lxu, LuuMT *const luu);

  void GetTerminalGradientHessian(const State &x, LxMT *const lx,
                                  LxxMT *const lxx);

  void AddCost(const BaseCostTerm *cost_term);
  void SetConstraintSize(const size_t &size);
  double GetAugLagCost(const State &x, const Control &u, const size_t &step);
  double GetTerminalConstraint(const State &x);
  void GetConstraintGradientHessian(const State &x, const Control &u,
                                    const size_t &step, LxMT *const lx,
                                    LuMT *const lu, LxxMT *const lxx,
                                    LxuMT *const lxu, LuuMT *const luu);
  void GetConstraintTerminalGradientHessian(const State &x, LxMT *const lx,
                                            LxxMT *const lxx);
  double UpateDynamics(StateVec *const x0, ControlVec &u0);
  void UpdateDuals(const State &x, const Control &u, const size_t &step);
  void UpdatePenalties(const size_t &step);
  void ResetDuals(const size_t &step);
  void ResetPenalties(const size_t &step);
  void AddConstraint(const size_t &horizon,
                     const std::vector<BoundConstraint> &constraint_vec);
  VectorXd GetDuals(const size_t &step);
  VectorXd GetPenalties(const size_t &step);
  bool CheckViolation(const size_t &step);
  VectorXd GetViolation(const size_t &step);
  size_t ConstraintSize() { return constraint_size_; }

  // init control u and u_vec
  void InitControlVar();

  void UpdateWarmStart(const std::vector<ControlVec> &warm_start_u_list) {
    warm_start_uk_list_.clear();
    warm_start_uk_list_.reserve(warm_start_u_list.size());
    for (const auto &element : warm_start_u_list) {
      warm_start_uk_list_.push_back(element);
    }
  }

  void SetCostMapPtr(std::vector<std::vector<double>> *cost_map_ptr) {
    cost_map_ptr_ = cost_map_ptr;
    cost_map_ptr->clear();
    cost_map_ptr_->resize(cost_stack_.size());
    for (auto &each_cost : *cost_map_ptr_) {
      each_cost.resize(GetHorizon() + 1);
    }
  }

  std::vector<const BaseCostTerm *> GetCostStackPtr() { return cost_stack_; }

  const ILqrModel *Model() const { return ilqr_model_ptr_; }

  size_t GetHorizon() const {
    return solver_config_ptr_->ilqr_config().horizon();
  }
  bool EnableCilqr() const { return solver_config_ptr_->enable_cilqr(); }
  bool IsDebugMode() const { return solver_config_ptr_->is_debug_mode(); }
  size_t GetInputSize() const {
    return solver_config_ptr_->ilqr_config().input_size();
  }
  size_t GetStateSize() const {
    return solver_config_ptr_->ilqr_config().state_size();
  }

 protected:
  std::vector<ControlVec> warm_start_uk_list_;

  // should be inited by core
  LuMT lu_;
  LxuMT lxu_;
  LuuMT luu_;

  const ILqrModel *ilqr_model_ptr_;

  // ilqr cost stack to restore all the costs
  std::vector<const BaseCostTerm *> cost_stack_;

  // ilqr cost stack to restore all the costs
  std::size_t constraint_size_{0};
  std::vector<std::vector<BoundConstraint>> constraint_horizan_;

  // solver config
  const SolverConfig *solver_config_ptr_;

  // cost map: total cost of each cost term
  std::vector<std::vector<double>> *cost_map_ptr_;

  // common terms
  BaseCostTerm *common_term_;
};
}  // namespace ilqr_solver
