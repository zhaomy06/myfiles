#include "ilqr_cost.h"

#include "constraint.h"
#include "ilqr_math_utils.h"

namespace ilqr_solver {

using npp::pnc::math_utils::ResizeAndResetEigen;
using npp::pnc::math_utils::ResizeAndResetEigenVec;

void CostManager::InitControlVar() {
  ResizeAndResetEigen(lu_, GetInputSize());
  ResizeAndResetEigen(lxu_, GetStateSize(), GetInputSize());
  ResizeAndResetEigen(luu_, GetInputSize(), GetInputSize());
}

double CostManager::UpateDynamics(StateVec *const x0, ControlVec &u0) {
  double cost = 0.0;
  for (size_t t = 0; t < GetHorizon(); ++t) {
    ilqr_model_ptr_->LimitControl((*x0)[t], &u0[t]);
    cost += GetCost((*x0)[t], u0[t], t);
    (*x0)[t + 1] = ilqr_model_ptr_->UpdateDynamicsOneStep((*x0)[t], u0[t], t);
  }
  cost += GetTerminalCost((*x0)[GetHorizon()]);
  std::cout << "cal init guess cost ......." << std::endl;
  return cost;
}

void CostManager::AddCost(const BaseCostTerm *cost_term) {
  cost_stack_.emplace_back(cost_term);
}

void CostManager::SetConstraintSize(const size_t &constraint_size) {
  constraint_size_ = constraint_size;
}

double CostManager::GetCost(const State &x, const Control &u,
                            const size_t &step) {
  common_term_->ComputeCommonTerms(x, u, step);

  double cost = 0.0;
  double result = 0.0;
  for (size_t i = 0; i < cost_stack_.size(); ++i) {
    result = cost_stack_[i]->GetCost(x, u, step);
    cost += result;
    // update cost map
    cost_map_ptr_->at(cost_stack_[i]->GetCostId())[step] = result;
  }
  return cost;
}

double CostManager::GetAugLagCost(const State &x, const Control &u,
                                  const size_t &step) {
  common_term_->ComputeCommonTerms(x, u, step);

  double augument_cost = 0.0;
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    augument_cost += constraint_horizan_[step][i].AugLag(x, u, step);
  }
  return augument_cost;
}

double CostManager::GetTerminalConstraint(const State &x) {
  VectorXd u = VectorXd::Zero(GetInputSize());
  const size_t step = GetHorizon();
  common_term_->ComputeCommonTerms(x, u, step);

  double constraint = 0.0;
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint += constraint_horizan_[GetHorizon()][i].AugLag(x, u, step);
  }
  return constraint;
}

void CostManager::GetConstraintTerminalGradientHessian(const State &x,
                                                       LxMT *const lx,
                                                       LxxMT *const lxx) {
  VectorXd u = VectorXd::Zero(GetInputSize());
  const size_t step = GetHorizon();
  common_term_->ComputeCommonTerms(x, u, step);
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[GetHorizon()][i].AugLagGradientHessian(
        x, u, step, lx, &lu_, lxx, &lxu_, &luu_);
  }
}

void CostManager::GetConstraintGradientHessian(
    const State &x, const Control &u, const size_t &step, LxMT *const lx,
    LuMT *const lu, LxxMT *const lxx, LxuMT *const lxu, LuuMT *const luu) {
  common_term_->ComputeCommonTerms(x, u, step);
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[step][i].AugLagGradientHessian(x, u, step, lx, lu, lxx,
                                                       lxu, luu);
  }
}

void CostManager::AddConstraint(
    const size_t &step, const std::vector<BoundConstraint> &constraint_vec) {
  constraint_horizan_.resize(GetHorizon() + 1);
  constraint_horizan_[step] = std::move(constraint_vec);
}

void CostManager::UpdateDuals(const State &x, const Control &u,
                              const size_t &step) {
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[step][i].UpdateDuals(x, u, step);
  }
}

void CostManager::UpdatePenalties(const size_t &step) {
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[step][i].UpdatePenalties();
  }
}

void CostManager::ResetDuals(const size_t &step) {
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[step][i].ResetDuals();
  }
}

void CostManager::ResetPenalties(const size_t &step) {
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    constraint_horizan_[step][i].ResetPenalties();
  }
}

VectorXd CostManager::GetDuals(const size_t &step) {
  VectorXd temp = VectorXd::Zero(ConstraintSize());
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    temp(i) = constraint_horizan_[step][i].GetDuals().cwiseAbs().maxCoeff();
  }
  return temp;
}

VectorXd CostManager::GetPenalties(const size_t &step) {
  VectorXd temp = VectorXd::Zero(ConstraintSize());
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    temp(i) = constraint_horizan_[step][i].GetPenalty();
  }
  return temp;
}

bool CostManager::CheckViolation(const size_t &step) {
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    if (!constraint_horizan_[step][i].CheckViolation()) {
      return false;
    }
  }
  return true;
}

VectorXd CostManager::GetViolation(const size_t &step) {
  VectorXd temp = VectorXd::Zero(ConstraintSize());
  for (size_t i = 0; i < ConstraintSize(); ++i) {
    temp(i) = constraint_horizan_[step][i].MaxViolation();
  }
  return temp;
}

double CostManager::GetTerminalCost(const State &x) {
  VectorXd u = VectorXd::Zero(GetInputSize());
  const size_t step = GetHorizon();
  common_term_->ComputeCommonTerms(x, u, step);

  double cost = 0.0;
  double result = 0.0;
  for (size_t i = 0; i < cost_stack_.size(); ++i) {
    result = cost_stack_[i]->GetCost(x, u, step);
    cost += result;

    // update cost map
    cost_map_ptr_->at(cost_stack_[i]->GetCostId())[GetHorizon()] = result;
  }
  return cost;
}

// Updates cx, cu
void CostManager::GetGradientHessian(const State &x, const Control &u,
                                     const size_t &step, LxMT *const lx,
                                     LuMT *const lu, LxxMT *const lxx,
                                     LxuMT *const lxu, LuuMT *const luu) {
  common_term_->ComputeCommonTerms(x, u, step);
  for (size_t i = 0; i < cost_stack_.size(); ++i) {
    cost_stack_[i]->GetGradientHessian(x, u, step, lx, lu, lxx, lxu, luu);
  }
}

void CostManager::GetTerminalGradientHessian(const State &x, LxMT *const lx,
                                             LxxMT *const lxx) {
  VectorXd u = VectorXd::Zero(GetInputSize());
  const size_t step = GetHorizon();
  common_term_->ComputeCommonTerms(x, u, step);
  for (size_t i = 0; i < cost_stack_.size(); ++i) {
    cost_stack_[i]->GetGradientHessian(x, u, step, lx, &lu_, lxx, &lxu_, &luu_);
  }
}

void CostManager::InitGuess(StateVec *const x0, ControlVec *const u0,
                            double *const init_cost,
                            const bool &update_init_seed,
                            int *init_seed_index) {
  if (update_init_seed || warm_start_uk_list_.empty()) {
    StateVec x;
    ResizeAndResetEigenVec(x, GetHorizon() + 1, GetStateSize());
    x[0] = (*x0)[0];
    *init_cost = std::numeric_limits<double>::max();
    *init_seed_index = -1;

    for (auto &uk_list : warm_start_uk_list_) {
      const double cost = UpateDynamics(&x, uk_list);
      if (cost < *init_cost) {
        (*init_seed_index)++;
        *init_cost = cost;
        *u0 = uk_list;
        *x0 = x;
      }
    }
  } else {
    *init_cost = UpateDynamics(x0, *u0);
  }
}

}  // namespace ilqr_solver
