#pragma once
#include <Eigen/Core>
#include <cstddef>
#include <deque>
#include <list>
#include <vector>
#include "virtual_lane_smooth.pb.h"

using State = Eigen::VectorXd;
using Control = Eigen::VectorXd;

enum StateId { X = 0, Y = 1, THETA = 2, KAPPA = 3,V =4,STATE_SIZE = 5 };
enum ControlId { DKAPPA = 0, A = 1, INPUT_SIZE = 2 };
enum RK4ID {
  k1_x = 0,
  k1_y = 1,
  k2_x = 2,
  k2_y = 3,
  k4_x = 4,
  k4_y = 5,
  RK4_SIZE
};

enum ILQRCostId : std::uint8_t {
  SMOOTH_COST=0,
  CONTROL_COST,
  TARGET_V_COST,
  REFLINE_COST,
  TERMINAL_COST,
  CORRIDOR_COST,
  LATERALACC_COST,
  COST_SIZE
};

enum FixedDataIndex {
  DELTA_T = 0,

  START_X,
  START_Y,
  START_THETA,
  W_SMOOTH_KAPPA,
  W_CONTROL_DKAPPA,
  W_CONTROL_A,
  W_TARGET_V,
  W_REFLINE,
  W_OBS,
  W_TERMINAL_POS,
  W_TERMINAL_THETA,
  W_LATERAL_ACC,
  TARGET_V,
  VEHICLE_LENGTH,
  VEHICLE_WIDTH,
  TERMINAL_X,
  TERMINAL_Y,
  TERMINAL_THETA,
  HORIZON,
  TO_TERMINAL,
  FIX_DATA_SIZE
};

enum StepDataIndex {
  CONTROL_INDEX,
  // previous point x,y
  PRE_X,
  PRE_Y,
  // state point lateral offset to refline
  DIS_LON,
  DISTORB,
  RB2POINTTHETA,
  // nearest obstacle position
  NEAREST_X,
  NEAREST_Y,
  // closet index to the terminal point
  CROSS_PROD,
  LON_DIS_SQURE,
  ANGLE_DIFF_SQURE,

  CLOSEST_INDEX,
  PRE_CLOSEST_INDEX,

  TERMINAL_FLAG,
  DIS_THRESHOLD_BY_STEP,
  HAS_NEAR_RB,
  IS_OUT_TERMINAL_RAY,
  IS_OUT_START_RAY,
  TERMINAL_RAY_PROJECTION_X,
  TERMINAL_RAY_PROJECTION_Y,
  START_RAY_PROJECTION_X,
  START_RAY_PROJECTION_Y,
  X_REF,
  Y_REF,
  STEP_DATA_SIZE
};

struct IlqrData {
  std::array<double, FIX_DATA_SIZE> fixed_data;
  std::vector<std::array<double, STEP_DATA_SIZE>> step_data;
  std::vector<RawPoint> raw_point_vec;
  std::vector<Eigen::Matrix<double, 4, 4>> rb_hPolys;
  size_t iter_count = 0;
};

class IlqrDataCache {
 public:
  IlqrDataCache() = default;
  const double &operator()(const FixedDataIndex index) const {
    return data_.fixed_data[index];
  }

  const double &operator()(const StepDataIndex index, const size_t step) const {
    return data_.step_data[step][index];
  }

  double *Mutable(const FixedDataIndex index) {
    return &data_.fixed_data[index];
  }

  double *Mutable(const StepDataIndex index, const size_t step) {
    return &data_.step_data[step][index];
  }

  void ResizeStepData(const size_t horizon) { data_.step_data.resize(horizon); }

  void UpdataIterCount(const size_t iter_count) {
    data_.iter_count = iter_count;
  }

  std::vector<RawPoint>* MutableRawPointVecPtr(){
    return &(data_.raw_point_vec);
  }

  const std::vector<RawPoint>& GetRawPointVec() const{
    return data_.raw_point_vec;
  }

  std::vector<Eigen::Matrix<double, 4, 4>>* MutableRBHPolysPtr(){
    return &(data_.rb_hPolys);
  }

  const std::vector<Eigen::Matrix<double, 4, 4>>& GetRBHPolys() const{
    return data_.rb_hPolys;
  }



  const size_t &GetIterCount() const { return data_.iter_count; }

 private:
    IlqrData data_;
};

