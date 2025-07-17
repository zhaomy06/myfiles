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
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "speed_planner/piecewise_jerk_speed_nonlinear_optimizer.h"

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "pnc_point.pb.h"
#include "speed_planner/common/util.h"
#include "speed_planner/piecewise_jerk_path_problem.h"
#include "speed_planner/piecewise_jerk_speed_problem.h"
#include "speed_planner/piecewise_jerk_speed_nonlinear_ipopt_interface.h"

namespace apollo {
namespace planning {

using apollo::common::SpeedPoint;

using apollo::common::TrajectoryPoint;

PiecewiseJerkSpeedNonlinearOptimizer::PiecewiseJerkSpeedNonlinearOptimizer(const SpeedPlannerConfig& config):
  smoothed_path_curvature_(0.0, 0.0, 0.0)
 {
  config_=config;
 }

bool PiecewiseJerkSpeedNonlinearOptimizer::Process(
    const PathData& path_data, const TrajectoryPoint& init_point) {


  if (path_data.discretized_path().empty()) {
    const std::string msg = "Speed Optimizer receives empty path data";
    std::cout << msg;
    return false;
  }

  bool problem_setups_status =
      SetUpStatesAndBounds(path_data,init_point);
  if (!problem_setups_status) {
    std::cout<<"SetUpStatesAndBounds fail"<<std::endl;
    return problem_setups_status;
  }
  else{
    std::cout<<"SetUpStatesAndBounds success"<<std::endl;
  }

  std::vector<double> distance;
  std::vector<double> velocity;
  std::vector<double> acceleration;

  const auto qp_start = std::chrono::system_clock::now();
  std::cout<<"start OptimizeByQP"<<std::endl;
  const auto qp_smooth_status =
      OptimizeByQP(&distance, &velocity, &acceleration);

  const auto qp_end = std::chrono::system_clock::now();
  std::chrono::duration<double> qp_diff = qp_end - qp_start;
  std::cout << "speed qp optimization takes " << qp_diff.count() * 1000.0 << " ms";

  if (!qp_smooth_status) {
    std::cout<<"OptimizeByQP fail"<<std::endl;
    return qp_smooth_status;
  }

  const auto curvature_smooth_start = std::chrono::system_clock::now();

  const auto path_curvature_smooth_status = SmoothPathCurvature(path_data);

  const auto curvature_smooth_end = std::chrono::system_clock::now();
  std::chrono::duration<double> curvature_smooth_diff =
      curvature_smooth_end - curvature_smooth_start;
  std::cout << "path curvature smoothing for nlp optimization takes "
          << curvature_smooth_diff.count() * 1000.0 << " ms";

  if (!path_curvature_smooth_status) {
    std::cout<<"path_curvature_smooth_status fail"<<std::endl;
    return path_curvature_smooth_status;
  }

  const auto nlp_start = std::chrono::system_clock::now();

  const auto nlp_smooth_status =
      OptimizeByNLP(&distance, &velocity, &acceleration);

  const auto nlp_end = std::chrono::system_clock::now();
  std::chrono::duration<double> nlp_diff = nlp_end - nlp_start;
  std::cout << "speed nlp optimization takes " << nlp_diff.count() * 1000.0
          << " ms";

  res_distance_=distance;
  res_velocity_=velocity;
  res_acceleration_=acceleration;

  if (!nlp_smooth_status) {
    std::cout<<"optimize by nlp fail"<<std::endl;
    return nlp_smooth_status;
  }

  return true;

}


bool PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByQP(
    std::vector<double>* distance,
    std::vector<double>* velocity, std::vector<double>* acceleration) {
  std::array<double, 3> init_states = {s_init_, s_dot_init_, s_ddot_init_};

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, delta_t_,
                                                   init_states);
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmax(s_dot_max_, init_states[1]));
  piecewise_jerk_problem.set_ddx_bounds(s_ddot_min_, s_ddot_max_);
  piecewise_jerk_problem.set_dddx_bound(s_dddot_min_, s_dddot_max_);
  piecewise_jerk_problem.set_x_bounds(s_bounds_);

  // TODO(Jinyun): parameter tunnings
  piecewise_jerk_problem.set_weight_x(1);
  piecewise_jerk_problem.set_weight_dx(1);
  piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());
  std::vector<double> x_ref;
  for (int i = 0; i < num_of_knots_; ++i) {
    const double curr_t = i * delta_t_;
    // get path_s
    x_ref.emplace_back(cruise_speed_*curr_t);
  }
  piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));

  // Solve the problem
  std::cout<<"start qp opt"<<std::endl;
  if (!piecewise_jerk_problem.Optimize()) {
    const std::string msg =
        "Speed Optimization by Quadratic Programming failed. ";
    std::cout << msg;
    return false;
  }

  *distance = piecewise_jerk_problem.opt_x();
  *velocity = piecewise_jerk_problem.opt_dx();
  *acceleration = piecewise_jerk_problem.opt_ddx();
  //   debug.PrintToLog();
  return true;
}


bool PiecewiseJerkSpeedNonlinearOptimizer::SetUpStatesAndBounds(
    const PathData& path_data,const TrajectoryPoint& init_point) {
  // Set st problem dimensions
  // TODO(Jinyun): move to confs
  delta_t_ = config_.delta_time();
  total_length_ = path_data.discretized_path().back().s();
  total_time_ = config_.total_time();
  num_of_knots_ = static_cast<int>(total_time_ / delta_t_) + 1;
  cruise_speed_=config_.target_v();
  std::cout<<"cruise_speed_="<<cruise_speed_<<std::endl;
  // Set initial values
  s_init_ = 0.0;
  s_dot_init_ = init_point.v();
  s_ddot_init_ = init_point.a();
  for(size_t i=0;i<num_of_knots_;++i){
    s_bounds_.push_back({0,total_length_*1.2});
  }

  // Set s_dot bounary
  s_dot_max_ = cruise_speed_*2;;

  // Set s_ddot boundary
  s_ddot_min_ = -1000;
  s_ddot_max_ = 1000;
  // Set s_dddot boundary
  // TODO(Jinyun): allow the setting of jerk_lower_bound and move jerk config_
  // to a better place
  s_dddot_min_ = -1000000;
  s_dddot_max_ = 1000000;

  return true;
}



bool PiecewiseJerkSpeedNonlinearOptimizer::SmoothPathCurvature(
    const PathData& path_data) {
  // using piecewise_jerk_path to fit a curve of path kappa profile
  // TODO(Jinyun): move smooth configs to gflags
  const auto& cartesian_path = path_data.discretized_path();
  const double delta_s = 0.5;
  std::vector<double> path_curvature;
  for (double path_s = cartesian_path.front().s();
       path_s < cartesian_path.back().s() + delta_s; path_s += delta_s) {
    const auto& path_point = cartesian_path.Evaluate(path_s);
    path_curvature.push_back(path_point.kappa());
  }
  const auto& path_init_point = cartesian_path.front();
  std::array<double, 3> init_state = {path_init_point.kappa(),
                                      path_init_point.dkappa(),
                                      path_init_point.ddkappa()};
  PiecewiseJerkPathProblem piecewise_jerk_problem(path_curvature.size(),
                                                  delta_s, init_state);
  piecewise_jerk_problem.set_x_bounds(-1.0, 1.0);
  piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
  piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

  piecewise_jerk_problem.set_weight_x(0.0);
  piecewise_jerk_problem.set_weight_dx(10.0);
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);

  piecewise_jerk_problem.set_x_ref(100.0, std::move(path_curvature));

  if (!piecewise_jerk_problem.Optimize(1000)) {
    const std::string msg = "Smoothing path curvature failed";
    std::cout << msg;
    return false;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_dx;
  std::vector<double> opt_ddx;

  opt_x = piecewise_jerk_problem.opt_x();
  opt_dx = piecewise_jerk_problem.opt_dx();
  opt_ddx = piecewise_jerk_problem.opt_ddx();

  PiecewiseJerkTrajectory1d smoothed_path_curvature(
      opt_x.front(), opt_dx.front(), opt_ddx.front());

  for (size_t i = 1; i < opt_ddx.size(); ++i) {
    double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
    smoothed_path_curvature.AppendSegment(j, delta_s);
  }

  smoothed_path_curvature_ = smoothed_path_curvature;

  return true;
}



bool PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByNLP(
    std::vector<double>* distance, std::vector<double>* velocity,
    std::vector<double>* acceleration) {
  // Set optimizer instance
  auto ptr_interface = new PiecewiseJerkSpeedNonlinearIpoptInterface(
      s_init_, s_dot_init_, s_ddot_init_, delta_t_, num_of_knots_,
      total_length_, s_dot_max_, s_ddot_min_, s_ddot_max_, s_dddot_min_,
      s_dddot_max_);
  ptr_interface->set_safety_bounds(s_bounds_);
  ptr_interface->set_curvature_curve(smoothed_path_curvature_);
  ptr_interface->set_tar_t_and_s(config_.target_time(),config_.target_s());
  if (config_.use_warm_start()) {
    const auto& warm_start_distance = *distance;
    const auto& warm_start_velocity = *velocity;
    const auto& warm_start_acceleration = *acceleration;
    if (warm_start_distance.empty() || warm_start_velocity.empty() ||
        warm_start_acceleration.empty() ||
        warm_start_distance.size() != warm_start_velocity.size() ||
        warm_start_velocity.size() != warm_start_acceleration.size()) {
      const std::string msg =
          "Piecewise jerk speed nonlinear optimizer warm start invalid!";
      std::cout << msg;
      return false;
    }
    std::vector<std::vector<double>> warm_start;
    std::size_t size = warm_start_distance.size();
    for (std::size_t i = 0; i < size; ++i) {
      warm_start.emplace_back(std::initializer_list<double>(
          {warm_start_distance[i], warm_start_velocity[i],
           warm_start_acceleration[i]}));
    }
    ptr_interface->set_warm_start(warm_start);
  }

  if (config_.use_smoothed_dp_guide_line()) {
    ptr_interface->set_reference_spatial_distance(*distance);
    // TODO(Jinyun): move to confs
    ptr_interface->set_w_reference_spatial_distance(10.0);
  } else {
    std::vector<double> spatial_potantial(num_of_knots_, total_length_);
    ptr_interface->set_reference_spatial_distance(spatial_potantial);
    ptr_interface->set_w_reference_spatial_distance(
        config_.s_potential_weight());
  }

  if (config_.use_soft_bound_in_nonlinear_speed_opt()) {
    ptr_interface->set_soft_safety_bounds(s_soft_bounds_);
    ptr_interface->set_w_soft_s_bound(config_.soft_s_bound_weight());
  }

  ptr_interface->set_w_overall_a(config_.acc_weight());
  ptr_interface->set_w_overall_j(config_.jerk_weight());
  ptr_interface->set_w_overall_centripetal_acc(config_.lat_acc_weight());

  ptr_interface->set_reference_speed(cruise_speed_);
  ptr_interface->set_w_reference_speed(config_.ref_v_weight());

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("max_iter", 1000);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    const std::string msg =
        "Piecewise jerk speed nonlinear optimizer failed during initialization";
    std::cout << msg;
    return false;
  }

  const auto start_timestamp = std::chrono::system_clock::now();
  status = app->OptimizeTNLP(problem);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "*** The optimization problem take time: " << diff.count() * 1000.0
         << " ms.";

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    std::cout << "*** The problem solved in " << iter_count << " iterations!";
    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    std::cout << "*** The final value of the objective function is " << final_obj
           << '.';
  } else {
    const auto& ipopt_return_status =
        false;

      std::cout << "Solver failure case is : " << ipopt_return_status;

    const std::string msg = "Piecewise jerk speed nonlinear optimizer failed";
    std::cout << msg;
    return false;
  }

  ptr_interface->get_optimization_results(distance, velocity, acceleration);

  //   debug.PrintToLog();
  return true;
}
}  // namespace planning
}  // namespace apollo
