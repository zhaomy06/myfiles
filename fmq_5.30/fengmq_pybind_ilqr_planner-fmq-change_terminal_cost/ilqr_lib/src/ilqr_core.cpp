#include "ilqr_core.h"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>

#include "ilqr_cost.h"

namespace ilqr_solver {

using npp::pnc::math_utils::ResetEigenVec;
using npp::pnc::math_utils::ResizeAndResetEigen;
using npp::pnc::math_utils::ResizeAndResetEigenMat;
using npp::pnc::math_utils::ResizeAndResetEigenVec;

void ILqr::Init(const ILqrModel *const model_ptr,
                const SolverConfig *const solver_config_ptr,
                BaseCostTerm *const common_term_calculator) {
  solver_config_ptr_ = solver_config_ptr;
  cost_manager_ptr_ = std::make_unique<CostManager>(
      model_ptr, solver_config_ptr, common_term_calculator);

  const size_t horizon = GetHorizon();
  const size_t input_size = GetInputSize();
  const size_t state_size = GetStateSize();

  ResizeAndResetEigenVec(xk_vec_, horizon + 1, state_size);
  ResizeAndResetEigenVec(xk_new_vec_, horizon + 1, state_size);
  ResizeAndResetEigenVec(uk_vec_, horizon, input_size);
  ResizeAndResetEigenVec(uk_new_vec_, horizon, input_size);
  ResizeAndResetEigenVec(lx_vec_, horizon + 1, state_size);
  ResizeAndResetEigenVec(lu_vec_, horizon, input_size);
  ResizeAndResetEigenMat(lxx_vec_, horizon + 1, state_size, state_size);
  ResizeAndResetEigenMat(lxu_vec_, horizon, state_size, input_size);
  ResizeAndResetEigenMat(luu_vec_, horizon, input_size, input_size);
  ResizeAndResetEigenMat(fx_vec_, horizon, state_size, state_size);
  ResizeAndResetEigenMat(fu_vec_, horizon, state_size, input_size);
  ResizeAndResetEigenVec(k_vec_, horizon, input_size);
  ResizeAndResetEigenMat(K_vec_, horizon, state_size, state_size);

  ResizeAndResetEigen(Qu_, input_size);
  ResizeAndResetEigen(Qx_, input_size);
  ResizeAndResetEigen(k_i_, input_size);
  ResizeAndResetEigen(Vx_, state_size);
  ResizeAndResetEigen(u_term_init_, input_size);
  ResizeAndResetEigen(Quu_, input_size, input_size);
  ResizeAndResetEigen(Qxx_, state_size, state_size);
  ResizeAndResetEigen(Qux_, input_size, state_size);
  ResizeAndResetEigen(Quuf_, input_size, input_size);
  ResizeAndResetEigen(K_i_, input_size, state_size);
  ResizeAndResetEigen(Quu_eye_, input_size, input_size);
  Quu_eye_.setIdentity();
  ResizeAndResetEigen(Vxx_, state_size, state_size);
  ResizeAndResetEigen(fxt_, state_size, state_size);
  ResizeAndResetEigen(fut_, input_size, state_size);
  ResizeAndResetEigen(fu_Vxx_, input_size, state_size);
  ResizeAndResetEigen(QuuF_inv_, input_size, input_size);
  ResizeAndResetEigen(Kt_Quu_Quxt_, state_size, input_size);
  ResizeAndResetEigen(Quxt_, state_size, input_size);
  ResizeAndResetEigen(Kt_Quu_, state_size, input_size);
  ResizeAndResetEigen(Kt_, state_size, input_size);

  // init cost size
  solver_info_.cost_size = 0;

  // for ilqr_model
  cost_manager_ptr_->InitControlVar();
}

void ILqr::InitIlqrSolverInfo(ILqrSolverInfo *ilqr_solver_info) const {
  ilqr_solver_info->solver_condition = iLqrSolveCondition::INIT;
  ilqr_solver_info->iter_count = 0;
  ilqr_solver_info->iteration_info_vec.clear();
  ilqr_solver_info->iteration_info_vec.resize(GetMaxIter() * GetMaxOutIter() +
                                              1);
}

void ILqr::AddCost(const BaseCostTerm *cost_term) {
  cost_manager_ptr_->AddCost(cost_term);
  solver_info_.cost_size++;
}

void ILqr::SetConstraintSize(const size_t &constraint_size) {
  cost_manager_ptr_->SetConstraintSize(constraint_size);
}

void ILqr::AddConstraint(const size_t &horizon,
                         const std::vector<BoundConstraint> &constraint_vec) {
  cost_manager_ptr_->AddConstraint(horizon, constraint_vec);
}

void ILqr::Simulation() {
  for (size_t i = 0; i < GetHorizon(); i++) {
    cost_manager_ptr_->Model()->LimitControl(xk_vec_[i], &(uk_vec_[i]));
    xk_vec_[i + 1] = cost_manager_ptr_->Model()->UpdateDynamicsOneStep(
        xk_vec_[i], uk_vec_[i], i);
  }
}

void ILqr::Solve(const State &x0) {
  // TODO: check feasibility of input and avoid coredump
  // InputFeasibilityCheck(xk, uk);

  // time tag of all start

  // init time info
  time_info_.Reset();
  time_info_.UpdateAllStart();
  InitIlqrSolverInfo(&solver_info_);

  // set init x
  xk_vec_[0] = x0;
  if (EnableCilqr()) {
    ResetPenalties();
    ResetDuals();
    ResetCilqrInfo();
  }

  // set first cost_map before init guess
  cost_manager_ptr_->SetCostMapPtr(
      &(solver_info_.iteration_info_vec[0].cost_map));

  // init guess
  bool update_init_seed = true;
  cost_manager_ptr_->InitGuess(&xk_vec_, &uk_vec_, &cost_, update_init_seed,
                               &init_seed_index_);
  if (EnableCilqr()) {
    for (size_t i = 0; i < GetHorizon(); ++i) {
      cost_ += cost_manager_ptr_->GetAugLagCost(xk_vec_[i], uk_vec_[i], i);
    }
    cost_ +=
        cost_manager_ptr_->GetTerminalConstraint(xk_new_vec_[GetHorizon()]);
  }

  // calculate t_init_guess_ms
  time_info_.t_init_guess_ms +=
      time_info_.GetElapsed(time_info_.all_start, false);

  // update advanced info after init guess
  UpdateAdvancedInfo(0);

  lambda_ = 0.0;
  lambda_gain_ = 1.0;

  // ilqr main iteration
  if (EnableCilqr()) {
    CILqrIteration();
  } else {
    ILqrIteration();
  }

  // t_one_step_ms
  ComputeTime();

  // print info when debug
  if (IsDebugMode()) {
    PrintSolverInfo();
    PrintCostInfo();
    PrintTimeInfo();
  }
}

void ILqr::Reset() { ResetEigenVec(uk_vec_); }

bool ILqr::PSDCheck(const Eigen::MatrixXd &Q) {
  if (GetInputSize() == 1) {
    // for scalar input, to make Quu be positive definite is very easy
    if (Q(0, 0) <= kLambdaMin) {
      lambda_ = kLambdaFix;
      return false;
    } else {
      return true;
    }
  } else {
    Eigen::LLT<Eigen::MatrixXd> lltOfA(Q);
    if (lltOfA.info() == Eigen::NumericalIssue) {
      return false;
    } else {
      return true;
    }
  }
}

/*
   INPUTS
      cx: 2x(N+1)          cu: 2x(N+1)
      cuu: nxnx(N+1)        cxx: nxnx(N+1)  cuu: 2x2x(N+1)
      fx: nxnx(N+1)        fu: nx2x(N+1)    fxx: none
      fxu: None            fuu: none        u: 2xT
    OUTPUTS
      Vx: nx(N+1)      Vxx: nxnx(N+1)      k:mxT
      K: mxnxT         dV: 2x1
      diverge - returns 0 if it doesn't diverge, timestep where it diverges
   otherwise
*/
bool ILqr::BackwardPass() {
  // cost-to-go at end
  std::cout << "backward pass" << std::endl;
  Vx_.noalias() = lx_vec_[GetHorizon()];
  Vxx_.noalias() = lxx_vec_[GetHorizon()];

  dV_.fill(0);

  for (int i = (static_cast<int>(GetHorizon()) - 1); i >= 0;
       i--) {  // back up from end of trajectory

    fut_.noalias() = fu_vec_[i].transpose();
    fxt_.noalias() = fx_vec_[i].transpose();
    fu_Vxx_.noalias() = fut_ * Vxx_;

    Qx_.noalias() = lx_vec_[i] + fxt_ * Vx_;
    Qu_.noalias() = lu_vec_[i] + fut_ * Vx_;
    Qxx_.noalias() = lxx_vec_[i] + fxt_ * Vxx_ * fx_vec_[i];
    Quu_.noalias() = luu_vec_[i] + fu_Vxx_ * fu_vec_[i];
    Qux_.noalias() = lxu_vec_[i].transpose() + fu_Vxx_ * fx_vec_[i];

    // Similar to equations 10a and 10b in [Tassa 2012]. Note that
    // regularization is different
    Quuf_.noalias() = Quu_ + lambda_ * Quu_eye_;
    if (!PSDCheck(Quuf_)) {
      return false;
    }

    QuuF_inv_.noalias() = Quuf_.inverse();
    k_i_.noalias() = -QuuF_inv_ * Qu_;
    K_i_.noalias() = -QuuF_inv_ * Qux_;

    Kt_.noalias() = K_i_.transpose();
    Kt_Quu_.noalias() = Kt_ * Quu_;
    Quxt_.noalias() = Qux_.transpose();
    Kt_Quu_Quxt_.noalias() = Kt_Quu_ + Quxt_;

    // Update cost-to-go approximation. Equations 11 in [Tassa 2012]
    dV_[0] += k_i_.transpose() * Qu_;
    dV_[1] += 0.5 * k_i_.transpose() * Quu_ * k_i_;

    Vx_.noalias() = Qx_ + Kt_Quu_Quxt_ * k_i_ + Kt_ * Qu_;
    Vxx_.noalias() = Qxx_ + Kt_Quu_Quxt_ * K_i_ + Kt_ * Qux_;

    // save controls/gains
    k_vec_[i].noalias() = k_i_;
    K_vec_[i].noalias() = K_i_;

    if (solver_config_ptr_->viz_matrix()) {
      std::cout << "i:" << i << ", lambda: " << lambda_
                << ", Qu: " << Qu_.transpose()
                << ", Quu_det: " << Quu_.determinant()
                << ", Quu: " << Quu_.transpose()
                << ", k_i_:" << k_i_.transpose()
                << ", K_i_:" << K_i_.transpose() << std::endl;
    }
  }
  return true;
}

// input x, u, dV, k
// output success_flag, new_cost
bool ILqr::ForwardPass(const size_t &iter, double *new_cost, double *expected) {
  xk_new_vec_ = xk_vec_;
  uk_new_vec_ = uk_vec_;
  // linesearch process
  for (size_t i = 0; i < alpha_vec.size(); ++i) {
    const double alpha = alpha_vec[i];
    std::cout << "forward pass , alpha = " << alpha << std::endl;
    // for (const auto &alpha : alpha_vec) {
    solver_info_.iteration_info_vec[iter].linesearch_count = i + 1;
    (*new_cost) = 0.0;

    for (size_t j = 0; j < GetHorizon(); ++j) {
      uk_new_vec_[j].noalias() = alpha * k_vec_[j] + uk_vec_[j] +
                                 K_vec_[j] * (xk_new_vec_[j] - xk_vec_[j]);
      cost_manager_ptr_->Model()->LimitControl(xk_new_vec_[j],
                                               &(uk_new_vec_[j]));
      if (solver_config_ptr_->viz_matrix()) {
        std::cout << "alpha: " << alpha << ", j:" << j
                  << ", uk_vec_:" << uk_vec_[j].transpose()
                  << ", k_vec_:" << k_vec_[j].transpose() << ", K_vec_:"
                  << (K_vec_[j] * (xk_new_vec_[j] - xk_vec_[j])).transpose()
                  << ", uk_new_vec_:" << uk_new_vec_[j].transpose()
                  << ", delta u: " << (uk_new_vec_[j] - uk_vec_[j]).transpose()
                  << std::endl;
      }

      if (EnableCilqr()) {
        (*new_cost) +=
            cost_manager_ptr_->GetAugLagCost(xk_new_vec_[j], uk_new_vec_[j], j);
      }
      xk_new_vec_[j + 1] = cost_manager_ptr_->Model()->UpdateDynamicsOneStep(
          xk_new_vec_[j], uk_new_vec_[j], j);
    }
    if (EnableCilqr()) {
      *new_cost +=
          cost_manager_ptr_->GetTerminalConstraint(xk_new_vec_[GetHorizon()]);
    }
    for (size_t j = 0; j < GetHorizon(); ++j) {
      (*new_cost) +=
          cost_manager_ptr_->GetCost(xk_new_vec_[j], uk_new_vec_[j], j);
    }

    (*new_cost) +=
        cost_manager_ptr_->GetTerminalCost(xk_new_vec_[GetHorizon()]);
    (*expected) = -alpha * (dV_[0] + alpha * dV_[1]);
    // note that (expected >= 0.0) does not mean reduced cost
    if ((*expected) >= 0.0) {
      if ((cost_ - (*new_cost)) / (*expected) > kZMin) {
        // line search success
        xk_vec_ = xk_new_vec_;
        uk_vec_ = uk_new_vec_;
        return true;
      }
      if (*expected < solver_config_ptr_->ilqr_config().cost_tol()) {
        *new_cost = cost_;
        solver_info_.solver_condition = EXPECTED_TOLERANCE;
        return true;
      }
    } else {
      // expected < 0 should not occour
      solver_info_.solver_condition = NON_POSITIVE_EXPECT;
      return false;
    }
  }
  return false;
}

void ILqr::IncreaseLambda() {
  lambda_gain_ = std::max(kLambdaFactor, lambda_gain_ * kLambdaFactor);
  lambda_ = std::max(kLambdaMin, lambda_ * lambda_gain_);
}

void ILqr::DecreaseLambda() {
  lambda_gain_ = std::min(1.0 / kLambdaFactor, lambda_gain_ / kLambdaFactor);
  lambda_ = lambda_ * lambda_gain_ * (lambda_ > kLambdaMin);
}

void ILqr::UpdateDynamicsDerivatives() {
  ResetEigenVec(lx_vec_);
  ResetEigenVec(lu_vec_);
  ResetEigenVec(lxx_vec_);
  ResetEigenVec(lxu_vec_);
  ResetEigenVec(luu_vec_);
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    if (i < GetHorizon()) {
      cost_manager_ptr_->Model()->GetDynamicsDerivatives(
          xk_vec_[i], uk_vec_[i], i, &fx_vec_[i], &fu_vec_[i]);
      cost_manager_ptr_->GetGradientHessian(
          xk_vec_[i], uk_vec_[i], i, &lx_vec_[i], &lu_vec_[i], &lxx_vec_[i],
          &lxu_vec_[i], &luu_vec_[i]);
      if (EnableCilqr()) {
        cost_manager_ptr_->GetConstraintGradientHessian(
            xk_vec_[i], uk_vec_[i], i, &lx_vec_[i], &lu_vec_[i], &lxx_vec_[i],
            &lxu_vec_[i], &luu_vec_[i]);
      }
      if (solver_config_ptr_->viz_matrix()) {
        std::cout << "i:" << i << ", fx_vec_:" << fx_vec_[i].transpose()
                  << ", fu_vec_:" << fu_vec_[i].transpose()
                  << ", lx_vec_:" << lx_vec_[i].transpose()
                  << ", lu_vec_:" << lu_vec_[i].transpose()
                  << ", lxx_vec_:" << lxx_vec_[i].transpose()
                  << ", lxu_vec_:" << lxu_vec_[i].transpose()
                  << ", luu_vec_:" << luu_vec_[i].transpose() << std::endl;
      }
    } else {
      cost_manager_ptr_->GetTerminalGradientHessian(xk_vec_[i], &lx_vec_[i],
                                                    &lxx_vec_[i]);
      if (EnableCilqr()) {
        cost_manager_ptr_->GetConstraintTerminalGradientHessian(
            xk_vec_[i], &lx_vec_[i], &lxx_vec_[i]);
      }
      if (solver_config_ptr_->viz_matrix()) {
        std::cout << "horizon i:" << i << ", lx_vec_:" << lx_vec_[i].transpose()
                  << ", lxx_vec_:" << lxx_vec_[i].transpose() << std::endl;
      }
    }
  }
}

// This assumes that x0, xs, us are initialized
bool ILqr::ILqrIteration() {
  if (xk_vec_.empty() || uk_vec_.empty()) {
    solver_info_.solver_condition = iLqrSolveCondition::FAULT_INPUT_SIZE;
    return false;
  }
  lambda_ = 0.0;
  lambda_gain_ = 1.0;

  if (EnableCilqr()) {
    // init guess
    bool update_init_seed = false;
    cost_manager_ptr_->InitGuess(&xk_vec_, &uk_vec_, &cost_, update_init_seed,
                                 &init_seed_index_);
    for (size_t i = 0; i < GetHorizon(); ++i) {
      cost_ += cost_manager_ptr_->GetAugLagCost(xk_vec_[i], uk_vec_[i], i);
    }
    cost_ +=
        cost_manager_ptr_->GetTerminalConstraint(xk_new_vec_[GetHorizon()]);
  }

  solver_info_.init_cost = cost_;

  // init some states
  solver_info_.solver_condition = iLqrSolveCondition::INIT;

  // Differentiate dynamics and cost along new trajectory before iteration
  ResetEigenVec(lx_vec_);
  ResetEigenVec(lu_vec_);
  ResetEigenVec(lxx_vec_);
  ResetEigenVec(lxu_vec_);
  ResetEigenVec(luu_vec_);

  // iteration loop
  bool solver_success = false;
  bool update_success = true;
  std::cout << "max iter = " << GetMaxIter() << std::endl;
  for (size_t iter = 0; iter < GetMaxIter(); ++iter) {
    // timer tag for iteration start
    time_info_.UpdateStart();

    // set iter count for info
    solver_info_.iter_count += 1;

    // set cost_map before iteration
    solver_info_.iteration_info_vec[solver_info_.iter_count].cost_map.resize(1);
    solver_info_.iteration_info_vec[solver_info_.iter_count].cost_map.clear();
    cost_manager_ptr_->SetCostMapPtr(
        &(solver_info_.iteration_info_vec[solver_info_.iter_count].cost_map));

    // update dynamics and derivatives when iter success
    if (update_success) {
      UpdateDynamicsDerivatives();
    } else {
      update_success = false;
    }

    // calculate t_compute_deriv_ms

    time_info_.t_compute_deriv_ms =
        time_info_.GetElapsed(time_info_.start, true);

    // STEP 1: backward pass
    size_t backward_pass_count = 0;
    while (true) {
      backward_pass_count++;
      // Update Vx, Vxx, l, L, dV with back_pass
      const bool is_converged = BackwardPass();
      if (is_converged) {
        break;
      } else {
        if (lambda_ > kLambdaMax ||
            backward_pass_count >= kMaxBackwardPassCount) {
          // backward pass failed, this should not happen when input_size = 1
          std::cout << "lambda: " << lambda_
                    << ", backward_pass_count: " << backward_pass_count
                    << std::endl;
          solver_info_.solver_condition =
              iLqrSolveCondition::BACKWARD_PASS_FAIL;
          return false;
        }
        IncreaseLambda();
      }
    }

    // calculate t_backward_pass_ms

    time_info_.t_backward_pass_ms =
        time_info_.GetElapsed(time_info_.start, true);

    // STEP 2: forward pass
    double expected = 0.0;
    double new_cost = 0.0;
    const bool forward_pass_success =
        ForwardPass(solver_info_.iter_count, &new_cost, &expected);
    const double dcost = cost_ - new_cost;

    auto &iter_debug_info =
        solver_info_.iteration_info_vec[solver_info_.iter_count];
    // solver info recording
    iter_debug_info.linesearch_success = forward_pass_success;

    iter_debug_info.cost = new_cost;
    iter_debug_info.dcost = dcost;
    iter_debug_info.lambda = lambda_;
    iter_debug_info.expect = expected;

    // calculate t_forward_pass_ms

    time_info_.t_forward_pass_ms +=
        time_info_.GetElapsed(time_info_.start, true);

    iter_debug_info.t_compute_deriv_ms = time_info_.t_compute_deriv_ms;
    iter_debug_info.t_backward_pass_ms = time_info_.t_backward_pass_ms;
    iter_debug_info.t_forward_pass_ms = time_info_.t_forward_pass_ms;

    // update advanced info after forward pass
    // +1 means considering init
    UpdateAdvancedInfo(solver_info_.iter_count);

    // STEP 3: check if terminate
    if (forward_pass_success) {
      // decrease lambda
      DecreaseLambda();

      // record last cost
      const double prev_iter_cost = cost_;

      // accept changes
      cost_ = new_cost;

      // when accept change, dynamics and derivatives should be updated
      update_success = true;
      if (solver_info_.solver_condition ==
          iLqrSolveCondition::EXPECTED_TOLERANCE) {
        solver_success = true;
        break;
      }
      // terminate check
      if (dcost < solver_config_ptr_->ilqr_config().cost_tol()) {
        solver_success = true;
        solver_info_.solver_condition =
            iLqrSolveCondition::NORMAL_COST_TOLERANCE;
        break;
      }

      if (dcost / prev_iter_cost <
          solver_config_ptr_->ilqr_config().cost_percent_tol()) {
        solver_success = true;
        solver_info_.solver_condition =
            iLqrSolveCondition::NORMAL_COST_PERCENT_TOLERANCE;
        break;
      }
    } else {
      // linesearch failed, increase lambda
      IncreaseLambda();

      // terminate?
      if (lambda_ > kLambdaMax) {
        solver_success = false;
        solver_info_.solver_condition = iLqrSolveCondition::LINESEARCH_FAIL;
        break;
      } else if (solver_info_.solver_condition ==
                 iLqrSolveCondition::NON_POSITIVE_EXPECT) {
        solver_success = false;
      } else {
        // should be considered next
        // when linesearch cannot result in reduced cost, terminate
        solver_success = true;
        if (iter > 0) {
          solver_info_.solver_condition =
              iLqrSolveCondition::MAX_LINESEARCH_TERMINATE;
        } else {
          solver_info_.solver_condition = iLqrSolveCondition::INIT_TERMINATE;
        }
        break;
      }
    }

    if (iter == GetMaxIter() - 1) {
      solver_info_.solver_condition = iLqrSolveCondition::MAX_ITER_TERMINATE;
      solver_success =
          true;  // This situation should not belong to solver success;
    }
  }  // end of iteration loop

  // force simulation
  // Simulation();

  return solver_success;
}

void ILqr::CILqrIteration() {
  // CILQR
  cilqr_info_.cilqr_solve.clear();
  for (int iter = 0;
       iter < solver_config_ptr_->cilqr_config().max_outer_iterations();
       ++iter) {
    ILqrIteration();
    UpdateDuals();
    UpdatePenalties();

    RecordCilqrInfo();

    if (cilqr_callback_ != nullptr) {
      cilqr_callback_(iter);
    }
    if (CheckViolation()) {
      cilqr_info_.iter_count = iter + 1;
      break;
    }
  }
}

void ILqr::UpdateAdvancedInfo(const size_t &iter) {
  // record x_vec and u_vec for each iteration
  solver_info_.iteration_info_vec[iter].x_vec = xk_vec_;
  solver_info_.iteration_info_vec[iter].u_vec = uk_vec_;
}

void ILqr::UpdateDuals() {
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    if (i < GetHorizon()) {
      cost_manager_ptr_->UpdateDuals(xk_vec_[i], uk_vec_[i], i);
    } else {
      cost_manager_ptr_->UpdateDuals(xk_vec_[i], u_term_init_, i);
    }
  }
}

void ILqr::UpdatePenalties() {
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    cost_manager_ptr_->UpdatePenalties(i);
  }
}

void ILqr::ResetPenalties() {
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    cost_manager_ptr_->ResetPenalties(i);
  }
}

void ILqr::ResetDuals() {
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    cost_manager_ptr_->ResetDuals(i);
  }
}

bool ILqr::CheckViolation() {
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    if (!cost_manager_ptr_->CheckViolation(i)) {
      return false;
    }
  }
  return true;
}

void ILqr::ComputeTime() {
  time_info_.t_one_step_ms = time_info_.GetElapsed(time_info_.all_start, false);
  solver_info_.total_time_ms = time_info_.t_one_step_ms;
  for (size_t i = 1; i < solver_info_.iter_count + 1; ++i) {
    solver_info_.t_compute_deriv_ms +=
        solver_info_.iteration_info_vec[i].t_compute_deriv_ms;
    solver_info_.t_backward_pass_ms +=
        solver_info_.iteration_info_vec[i].t_backward_pass_ms;
    solver_info_.t_forward_pass_ms +=
        solver_info_.iteration_info_vec[i].t_forward_pass_ms;
  }
}

void ILqr::RecordCilqrInfo() {
  CilqrIteration cilqr_iteration_tmp;
  std::size_t constraint_size = cost_manager_ptr_->ConstraintSize();
  VectorXd lambda_value = VectorXd::Zero(constraint_size);
  VectorXd penalty_value = VectorXd::Zero(constraint_size);
  VectorXd violation = VectorXd::Zero(constraint_size);
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    CilqrInfo cilqr_info_tmp;
    lambda_value = cost_manager_ptr_->GetDuals(i);
    penalty_value = cost_manager_ptr_->GetPenalties(i);
    VectorXd temp_violation = cost_manager_ptr_->GetViolation(i);

    for (size_t j = 0; j < constraint_size; ++j) {
      cilqr_info_tmp.cilqr_info.emplace_back(lambda_value(j), penalty_value(j));
      if (temp_violation(j) > violation(j)) {
        violation(j) = temp_violation(j);
      }
    }
    cilqr_iteration_tmp.cilqr_iteration.emplace_back(cilqr_info_tmp);
  }

  for (size_t i = 0; i < constraint_size; ++i) {
    cilqr_iteration_tmp.max_violation.emplace_back(violation(i));
  }
  cilqr_info_.cilqr_solve.emplace_back(cilqr_iteration_tmp);
}

void ILqr::ResetCilqrInfo() {
  CilqrIteration cilqr_iteration_tmp;
  std::size_t constraint_size = cost_manager_ptr_->ConstraintSize();
  for (size_t i = 0; i < GetHorizon() + 1; ++i) {
    CilqrInfo cilqr_info_tmp;
    for (size_t j = 0; j < constraint_size; ++j) {
      cilqr_info_tmp.cilqr_info.emplace_back(0.0, 0.0);
    }
    cilqr_iteration_tmp.cilqr_iteration.emplace_back(cilqr_info_tmp);
  }
  for (size_t i = 0; i < constraint_size; ++i) {
    cilqr_iteration_tmp.max_violation.emplace_back(0.0);
  }
  cilqr_info_.cilqr_solve.emplace_back(cilqr_iteration_tmp);
}

void ILqr::PrintSolverInfo() {
  if (!IsDebugMode()) {
    return;
  }
  std::cout
      << "--------------------------------------------------- ILqr solver info "
         "--------------------------------------------------- "
      << std::endl;

  // iteration info
  std::cout << "cost size = " << solver_info_.cost_size
            << ", init cost = " << solver_info_.init_cost
            << ", iteration count = " << solver_info_.iter_count
            << ", solver condition = "
            << static_cast<size_t>(solver_info_.solver_condition) << std::endl;

  std::cout
      << "iteration\tcost\t\treduction\t\texpect\t\t\tlambda\t\tLS_count\tLS_"
         "success\n";

  for (size_t iter = 0; iter < solver_info_.iter_count; ++iter) {
    printf("%-12ld\t%-12.10g\t%-12.15f\t%-12.15f\t%-12.5f\t%-12ld\t%-12d\n",
           iter, solver_info_.iteration_info_vec[iter].cost,
           solver_info_.iteration_info_vec[iter].dcost,
           solver_info_.iteration_info_vec[iter].expect,
           solver_info_.iteration_info_vec[iter].lambda,
           solver_info_.iteration_info_vec[iter].linesearch_count,
           solver_info_.iteration_info_vec[iter].linesearch_success);
  }
}

void ILqr::PrintCostInfo() {
  if (!IsDebugMode()) {
    return;
  }
  // cost info
  std::cout << "-----enable_cilqr: " << EnableCilqr() << std::endl;
  if (EnableCilqr()) {
    std::cout << "-----cilqr_iteration: " << cilqr_info_.cilqr_solve.size()
              << std::endl;
    std::cout << "-----cilqr_check_violation: " << CheckViolation()
              << std::endl;
  }
  std::cout << "\n-----cost vec info:" << std::endl;
  const auto cost_size = solver_info_.cost_size;
  std::vector<std::vector<double>> cost_vec_vec(
      solver_info_.iter_count + 1, std::vector<double>(cost_size, 0.0));

  for (size_t i = 0; i < cost_vec_vec.size(); ++i) {
    const auto &cost_map = solver_info_.iteration_info_vec[i].cost_map;
    for (size_t j = 0; j < cost_size; ++j) {
      for (const double cost : cost_map[j]) {
        cost_vec_vec[i][j] += cost;
      }
    }
  }

  for (size_t iter = 0; iter < cost_vec_vec.size(); ++iter) {
    if (iter == 0) {
      printf("cost_vec[init] = [ ");
    } else {
      printf("cost_vec[%ld]    = [ ", iter - 1);
    }

    for (const auto cost : cost_vec_vec[iter]) {
      printf("%.5f ", cost);
    }
    printf("]\n");
  }
  std::cout << std::endl;
}

void ILqr::PrintTimeInfo() {
  if (!IsDebugMode()) {
    return;
  }
  // time info
  std::cout << "\n-----time info:" << std::endl;
  std::cout << "Total time: " << time_info_.t_one_step_ms << std::endl;
  std::cout << "compute_derivatives: " << time_info_.t_compute_deriv_ms
            << std::endl;
  std::cout << "backward pass: " << time_info_.t_backward_pass_ms << std::endl;
  std::cout << "forward pass: " << time_info_.t_forward_pass_ms << std::endl;
  std::cout << "other stuff: "
            << time_info_.t_one_step_ms - (time_info_.t_compute_deriv_ms +
                                           time_info_.t_backward_pass_ms +
                                           time_info_.t_forward_pass_ms)
            << std::endl;
  // record time cost info
  solver_info_.total_time_ms = time_info_.t_one_step_ms;
}
}  // namespace ilqr_solver
