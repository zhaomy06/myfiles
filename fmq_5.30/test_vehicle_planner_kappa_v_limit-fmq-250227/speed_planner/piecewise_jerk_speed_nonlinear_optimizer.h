/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file piecewise_jerk_speed_nonlinear_optimizer.h
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "pnc_point.pb.h"
#include "piecewise_jerk_speed_nonlinear.pb.h"
#include "speed_planner/piecewise_jerk_trajectory1d.h"
#include "speed_planner/common/path_data.h"
#include "speed_planner_config.pb.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedNonlinearOptimizer{
 public:
  PiecewiseJerkSpeedNonlinearOptimizer(const SpeedPlannerConfig& config);

  virtual ~PiecewiseJerkSpeedNonlinearOptimizer() = default;

  bool Process(const ::apollo::planning::PathData& path_data,
                         const apollo::common::TrajectoryPoint& init_point) ;

  const std::vector<double>& GetResDistance(){
    return res_distance_;
  }

  const std::vector<double>& GetResVelocity(){
    return res_velocity_;
  }

  const std::vector<double>& GetResAcceleration(){
    return res_acceleration_;
  }

  private:
  bool OptimizeByQP(
    std::vector<double>* distance,
    std::vector<double>* velocity, std::vector<double>* acceleration);

  bool SetUpStatesAndBounds(const PathData& path_data, const apollo::common::TrajectoryPoint& init_point);


  bool SmoothPathCurvature(const PathData& path_data);


  bool OptimizeByNLP(std::vector<double>* distance,
                               std::vector<double>* velocity,
                               std::vector<double>* acceleration);

  // st problem dimensions
  double delta_t_ = 0.0;
  double total_length_ = 0.0;
  double total_time_ = 0.0;
  int num_of_knots_ = 0;

  // st initial values
  double s_init_ = 0.0;
  double s_dot_init_ = 0.0;
  double s_ddot_init_ = 0.0;

  // st states dynamically feasibility bounds
  double s_dot_max_ = 0.0;
  double s_ddot_min_ = 0.0;
  double s_ddot_max_ = 0.0;
  double s_dddot_min_ = 0.0;
  double s_dddot_max_ = 0.0;

  // st safety bounds
  std::vector<std::pair<double, double>> s_bounds_;
  std::vector<std::pair<double, double>> s_soft_bounds_;

  // smoothed path curvature profile as a function of traversal distance
  PiecewiseJerkTrajectory1d smoothed_path_curvature_;

  // reference speed profile


  // reference cruise speed
  double cruise_speed_;
  SpeedPlannerConfig config_;
  std::vector<double> res_distance_;
  std::vector<double> res_velocity_;
  std::vector<double> res_acceleration_;
};


}  // namespace planning
}  // namespace apollo
