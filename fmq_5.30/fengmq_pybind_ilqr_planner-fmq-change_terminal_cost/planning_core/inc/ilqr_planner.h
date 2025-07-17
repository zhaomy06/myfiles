// #pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include "planning_core/inc/ilqr_planner_data.h"
#include "planning_core/inc/math_utils.h"
#include "ilqr_lib/inc/ilqr_core.h"
#include "ilqr_lib/inc/ilqr_cost.h"
#include "ilqr_lib/inc/ilqr_model.h"
#include "ilqr_data.pb.h"
#include "virtual_lane_smooth.pb.h"
#include "ilqr_lib/inc/line_segment2d.h"


class IlqrPlannerModel : public ilqr_solver::ILqrModel {
 public:
  explicit IlqrPlannerModel(const IlqrDataCache &data) : data_(data) {}
  State UpdateDynamicsOneStep(const State &x, const Control &u,
                              const size_t &step) const override;

  void GetDynamicsDerivatives(const State &x, const Control &u,
                              const size_t &step, FxMT *const f_x,
                              FuMT *const f_u) const override;

  static State UpdateDynamicsWithDt(const State &x, const Control &u,
                                    const double &dt);

  void LimitControl(const State &x, Control *const u) const override;

  static void CalcRK4(const double &theta, const double &kappa,
                      const double &dkappa, const double &ds,
                      std::vector<double> &k);

 private:
  const IlqrDataCache &data_;
};

class IlqrCommonTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit IlqrCommonTerm(const IlqrParams &params,
                                 IlqrDataCache *data)
      : ilqr_params_(params), data_(data) {}
  void ComputeCommonTerms(const State &x, const Control & /*u*/,
                          const size_t step) override;
  void SetIlqrCore(ilqr_solver::ILqr *const ilqr_core_ptr) {
    ilqr_core_ptr_ = ilqr_core_ptr;
  }
  void SetInitState(StateVec state_vec) { state_vec_ = state_vec; }
  void GetNearestIndex(const StateVec *state_vec_ptr, int &nearest_index);
  void RollBackInfo(const State &x, const size_t step);

 private:
  const IlqrParams &ilqr_params_;
  IlqrDataCache *data_;
  StateVec state_vec_;
  ilqr_solver::ILqr *ilqr_core_ptr_;
};


class SmoothCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit SmoothCostTerm(const IlqrParams &ilqr_params,
                          const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}
  double GetCost(const State &x, const Control &u,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control &u, const size_t step,
                          LxMT *const lx, LuMT *const lu, LxxMT *const lxx,
                          LxuMT *const lxu,
                          LuuMT *const /*luu*/) const override;
  std::uint8_t GetCostId() const override { return SMOOTH_COST; }
 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class ControlCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit ControlCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & /*x*/, const Control &u,
                 const size_t step) const override;
  void GetGradientHessian(const State & /*x*/, const Control &u,
                          const size_t step, LxMT *const /*lx*/, LuMT *const lu,
                          LxxMT *const /*lxx*/, LxuMT *const /*lxu*/,
                          LuuMT *const luu) const override;
  std::uint8_t GetCostId() const override { return CONTROL_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class ReflineCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit ReflineCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & x, const Control &/*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State & x, const Control &/*u*/,
                          const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                          LxxMT *const lxx, LxuMT *const /*lxu*/,
                          LuuMT *const /*luu*/) const override;
  std::uint8_t GetCostId() const override { return REFLINE_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class TerminalCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit TerminalCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & x, const Control &/*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State & x, const Control &/*u*/,
                          const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                          LxxMT *const lxx, LxuMT *const /*lxu*/,
                          LuuMT *const /*luu*/) const override;
  std::uint8_t GetCostId() const override { return TERMINAL_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class LateralAccCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit LateralAccCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & x, const Control &/*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State & x, const Control &/*u*/,
                          const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                          LxxMT *const lxx, LxuMT *const /*lxu*/,
                          LuuMT *const /*luu*/) const override;
  std::uint8_t GetCostId() const override { return LATERALACC_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class CorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit CorridorCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & x, const Control &/*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State & x, const Control &/*u*/,
                          const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                          LxxMT *const lxx, LxuMT *const /*lxu*/,
                          LuuMT *const /*luu*/) const override;
  void GetVehicleCornerPt(std::vector<State>& vehicle_corner_pts,std::vector<Eigen::Vector2d>& raw_corner,const State & x) const;
  std::uint8_t GetCostId() const override { return CORRIDOR_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};

class TargetVCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  explicit TargetVCostTerm(const IlqrParams &ilqr_params,
                           const IlqrDataCache &data)
      : ilqr_params_(ilqr_params), data_(data) {}

  double GetCost(const State & /*x*/, const Control &u,
                 const size_t step) const override;
  void GetGradientHessian(const State & /*x*/, const Control &u,
                          const size_t step, LxMT *const /*lx*/, LuMT *const lu,
                          LxxMT *const /*lxx*/, LxuMT *const /*lxu*/,
                          LuuMT *const luu) const override;
  std::uint8_t GetCostId() const override { return TARGET_V_COST; }

 private:
  const IlqrParams &ilqr_params_;
  const IlqrDataCache &data_;
};


class IlqrOptimizer {
 public:
  IlqrOptimizer() = default;
  explicit IlqrOptimizer(const IlqrParams &params)
      : ilqr_params_(params) {}
  ~IlqrOptimizer() = default;
  bool Process(IlqrProtoMsg &proto_data);
  void Init();
  void Update();
  void PrintDebugMsg();
 
 private:
  void Init(const ilqr_solver::SolverConfig& solver_config);
  void Update(const IlqrWarmStartData& ilqr_warm_start_data);
  void UpdateCostConfig();
  void UpdateInitState(const IlqrState& init_state, State *x );
  void UpdateWarmStartData(const IlqrWarmStartData& ilqr_warm_start_data);
  void Update_params();
  void CalcWarmStart(
    const IlqrProtoMsg &proto_msg, const double &dt,
    IlqrWarmStartData* warm_start_data);
  void Solve() const;
  size_t Horizon() const { return SolverConfig().ilqr_config().horizon(); }
  // double Deltas() const { return SolverConfig().model_dt; };
  const ilqr_solver::SolverConfig &SolverConfig() const {
    return ilqr_core_ptr_->GetSolverConfig();
  }
  void MatchStartIndex();
  enum ExtendDir { LEFT = 0, TOP, RIGHT, BOTTOM, EXTEND_DIR_SIZE };
using ExtendFinishTable = std::array<bool, 4>;
std::array<std::pair<int, int>, EXTEND_DIR_SIZE> col_indices = {
    {{BOTTOM, LEFT}, {LEFT, TOP}, {TOP, RIGHT}, {RIGHT, BOTTOM}}};
  void GetRectangleConst(
    std::vector<Eigen::Matrix<double, 4, 4>>* rb_hPolys,const IlqrProtoMsg &proto_msg) ;
  bool IsCollision(const Box2d& box) const;
  bool IsCollision(const LineSegment2d& line) const;
  void AddProtoObb();
  bool IsExentFinish(const ExtendFinishTable table);

 private:
  StateVec x0_{};
  IlqrParams ilqr_params_{};

  std::unique_ptr<ilqr_solver::SolverConfig> ilqr_solver_config_ptr_{nullptr};
  std::unique_ptr<ilqr_solver::ILqr> ilqr_core_ptr_{nullptr};

  std::unique_ptr<IlqrDataCache> data_{nullptr};
  IlqrProtoMsg* proto_msg_;
  std::vector<std::unique_ptr<ilqr_solver::BaseCostTerm>> cost_stack_vec_;
  std::unique_ptr<IlqrCommonTerm> common_term_calculator_{nullptr};
  std::unique_ptr<IlqrPlannerModel> planner_model_{nullptr};
  std::vector<Obbox> obb_list;
  int start_index_;
  int end_index_;
  int warm_start_end_idx;

};
