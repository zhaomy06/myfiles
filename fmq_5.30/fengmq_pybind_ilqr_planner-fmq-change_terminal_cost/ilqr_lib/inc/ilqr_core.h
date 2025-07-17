#pragma once
#include <array>
#include <memory>

#include "bound_constraint.h"
#include "constraint.h"
#include "ilqr_cost.h"
#include "ilqr_data.pb.h"
#include "ilqr_math_utils.h"
#include "ilqr_model.h"

namespace ilqr_solver {

using CILQRCallback = std::function<void(const std::size_t &)>;

class ILqr {
 public:
  // ilqr solver condition of result
  enum iLqrSolveCondition {
    INIT,                           // 0 solver init
    NORMAL_COST_TOLERANCE,          // 1 normal terminate, d_cost < cost_tol
    NORMAL_COST_PERCENT_TOLERANCE,  // 2 normal terminate, d_cost / pre_cost <
                                    // cost_percent_tol
    MAX_LINESEARCH_TERMINATE,       // 3 max linesearch in forward pass
    MAX_ITER_TERMINATE,             // 4 max iteration
    INIT_TERMINATE,                 // 5 one step iteration, init is the minimum
    BACKWARD_PASS_FAIL,   // 6 backward pass failed, non-positive definite Quu
    NON_POSITIVE_EXPECT,  // 7 non-positive expect, should not happen
    LINESEARCH_FAIL,      // 8 linesearch fail, lambda is too big
    FAULT_INPUT_SIZE,     // 9 input size is error
    EXPECTED_TOLERANCE
  };

  // solver info for each iteration
  struct IterationInfo {
    bool linesearch_success = false;
    size_t linesearch_count = 0;
    double lambda = 0.0;
    double cost = 0.0;
    double dcost = 0.0;
    double expect = 0.0;
    StateVec x_vec;
    ControlVec u_vec;
    double t_compute_deriv_ms = 0.0;
    double t_backward_pass_ms = 0.0;
    double t_forward_pass_ms = 0.0;
    std::vector<std::vector<double>> cost_map;
  };

  // solver info for all iterations
  struct ILqrSolverInfo {
    uint8_t solver_condition = NORMAL_COST_TOLERANCE;
    size_t cost_size = 0;
    size_t iter_count = 0;
    double init_cost = 0.0;
    double total_time_ms = 0.0;
    double t_compute_deriv_ms = 0.0;
    double t_backward_pass_ms = 0.0;
    double t_forward_pass_ms = 0.0;
    std::vector<IterationInfo> iteration_info_vec;
  };

  struct CilqrElem {
    CilqrElem(double lambda_, double penalty_)
        : lambda(lambda_), penalty(penalty_) {}
    double lambda = 0.0;
    double penalty = 0.0;
  };

  struct CilqrInfo {
    std::vector<CilqrElem> cilqr_info;
  };

  struct CilqrIteration {
    std::vector<CilqrInfo> cilqr_iteration;
    std::vector<double> max_violation;
  };

  struct CilqrSolve {
    std::vector<CilqrIteration> cilqr_solve;
    size_t iter_count = 0;
  };

  struct TimeInfo {
    void Reset() {
      t_init_guess_ms = 0.0;
      t_compute_deriv_ms = 0.0;
      t_backward_pass_ms = 0.0;
      t_forward_pass_ms = 0.0;
      t_one_step_ms = 0.0;
    }

    double GetElapsed(std::chrono::time_point<std::chrono::system_clock> &start,
                      const bool is_overlay) {
      auto now = std::chrono::system_clock::now();
      auto elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - start)
              .count();

      if (is_overlay) {
        start = now;
      }

      return elapsed * 1e-6;
    }

    void UpdateAllStart() { all_start = std::chrono::system_clock::now(); }

    void UpdateStart() { start = std::chrono::system_clock::now(); }

    std::chrono::time_point<std::chrono::system_clock> all_start;
    std::chrono::time_point<std::chrono::system_clock> start;
    double t_init_guess_ms = 0.0;
    double t_compute_deriv_ms = 0.0;
    double t_backward_pass_ms = 0.0;
    double t_forward_pass_ms = 0.0;
    double t_one_step_ms = 0.0;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ILqr() = default;
  ~ILqr() = default;

  void Init(const ILqrModel *const model_ptr,
            const SolverConfig *const solver_config_ptr,
            BaseCostTerm *const common_term_calculator);

  // usually do not need init cost
  void Solve(const State &x0);
  void UpdateDuals();
  void UpdatePenalties();
  void ResetPenalties();
  void ResetDuals();
  bool CheckViolation();
  void RecordCilqrInfo();
  void ResetCilqrInfo();

  void UpdateWarmStart(const std::vector<ControlVec> &warm_start_u_list) {
    cost_manager_ptr_->UpdateWarmStart(warm_start_u_list);
  }
  // void InputFeasibilityCheck(const StateVec &xk, const ControlVec &uk);

  const StateVec *GetStateResultPtr() const { return &xk_vec_; }

  const StateVec *GetNextStateResultPtr() const { return &xk_new_vec_; }

  const ControlVec *GetControlResultPtr() const { return &uk_vec_; }

  const ILqrSolverInfo *GetSolverInfoPtr() { return &solver_info_; }

  const CilqrSolve *GetCilqrInfoPtr() { return &cilqr_info_; }
  const TimeInfo *GetTimeInfoPtr() { return &time_info_; }

  void PrintSolverInfo();
  void PrintCostInfo();
  void PrintTimeInfo();

  void UpdateDynamicsDerivatives();
  void AddCost(const BaseCostTerm *cost_term);
  void SetConstraintSize(const size_t &constraint_size);
  void AddConstraint(const size_t &horizon,
                     const std::vector<BoundConstraint> &constraint_vec);

  void SetCallback(CILQRCallback callback) { cilqr_callback_ = callback; }

  const SolverConfig &GetSolverConfig() { return *solver_config_ptr_; }

  // update cost_vec
  void UpdateAdvancedInfo(const size_t &iter);

  // reset solver
  void Reset();
  void InitIlqrSolverInfo(ILqrSolverInfo *ilqr_solver_info) const;

  size_t GetHorizon() const {
    return solver_config_ptr_->ilqr_config().horizon();
  }
  size_t GetMaxIter() const {
    return solver_config_ptr_->ilqr_config().max_iter();
  }
  size_t GetMaxOutIter() const {
    return solver_config_ptr_->cilqr_config().max_outer_iterations();
  }
  int GetInitSeedIndex() const { return init_seed_index_; }
  bool EnableCilqr() const { return solver_config_ptr_->enable_cilqr(); }
  bool IsDebugMode() const { return solver_config_ptr_->is_debug_mode(); }
  size_t GetInputSize() const {
    return solver_config_ptr_->ilqr_config().input_size();
  }
  size_t GetStateSize() const {
    return solver_config_ptr_->ilqr_config().state_size();
  }
  void ComputeTime();

 private:
  bool BackwardPass();
  bool ILqrIteration();
  void CILqrIteration();

  bool ForwardPass(const size_t &iter, double *new_cost, double *expected);
  void IncreaseLambda();
  void DecreaseLambda();
  bool PSDCheck(const Eigen::MatrixXd &Q);
  void Simulation();

  StateVec xk_vec_;
  ControlVec uk_vec_;
  StateVec xk_new_vec_;
  ControlVec uk_new_vec_;
  Control u_term_init_;

  double cost_ = 0.0;

  LxMTVec lx_vec_;    // n * (CONTROL_HORIZON)
  LuMTVec lu_vec_;    // m * (CONTROL_HORIZON)
  LxxMTVec lxx_vec_;  // n * n * (CONTROL_HORIZON)
  LxuMTVec lxu_vec_;  // n * m * (CONTROL_HORIZON)
  LuuMTVec luu_vec_;  // m * m * (CONTROL_HORIZON)

  FxMTVec fx_vec_;  // n * n * (CONTROL_HORIZON)
  FuMTVec fu_vec_;  // n * m * (CONTROL_HORIZON)

  std::array<double, 2> dV_{};  // 2 * 1
  kMTVec k_vec_;                // m * (CONTROL_HORIZON + 1)
  KMTVec K_vec_;                // m * n * (CONTROL_HORIZON + 1)

  Eigen::VectorXd Qu_, Qx_, k_i_;
  Eigen::MatrixXd Quu_, Qxx_, Qux_, Quuf_, K_i_;
  Eigen::MatrixXd Quu_eye_;

  Eigen::VectorXd Vx_;
  Eigen::MatrixXd Vxx_, fxt_, fut_, fu_Vxx_, QuuF_inv_, Kt_Quu_Quxt_, Quxt_,
      Kt_Quu_, Kt_;

  double lambda_ = 0.0;
  double lambda_gain_ = 1.0;
  int init_seed_index_ = -1;

  // model
  std::unique_ptr<CostManager> cost_manager_ptr_;

  // solver config
  const SolverConfig *solver_config_ptr_;

  // solver info
  ILqrSolverInfo solver_info_;
  CilqrSolve cilqr_info_;

  // time debug info
  TimeInfo time_info_;

  // extra callback
  CILQRCallback cilqr_callback_{nullptr};
};
}  // namespace ilqr_solver
