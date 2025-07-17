#include "planning_core/inc/ilqr_planner.h"
#include <cmath>
#include<algorithm>


//IlqrOptimizer
bool IlqrOptimizer::Process(IlqrProtoMsg &proto_msg){
  proto_msg_=&proto_msg;
  data_ = std::make_unique<IlqrDataCache>();
  MatchStartIndex();
  AddProtoObb();
  std::cout<<"match start index="<<start_index_<<std::endl;
  std::cout<<"match end index="<<end_index_<<std::endl;
  planner_model_ = std::make_unique<IlqrPlannerModel>(*data_);
  Update_params();
  IlqrWarmStartData ilqr_warm_start_data{};
  CalcWarmStart(proto_msg,(*data_)(DELTA_T),&ilqr_warm_start_data);
  proto_msg.mutable_warm_start_data()->CopyFrom(ilqr_warm_start_data);
  std::vector<Eigen::Matrix<double, 4, 4>> rb_hPolys{};
  GetRectangleConst(&rb_hPolys,proto_msg);
  *(data_->MutableRBHPolysPtr())=rb_hPolys;
  std::cout<<"len of rb_hPolys="<<rb_hPolys.size()<<std::endl;
    // set solver config
  ilqr_solver::SolverConfig ilqr_solver_config{};
  ilqr_solver_config.mutable_ilqr_config()->set_horizon(
      ilqr_warm_start_data.controls_size());
  ilqr_solver_config.mutable_ilqr_config()->set_state_size(
      static_cast<size_t>(StateId::STATE_SIZE));
  ilqr_solver_config.mutable_ilqr_config()->set_input_size(
      static_cast<size_t>(ControlId::INPUT_SIZE));
  ilqr_solver_config.mutable_ilqr_config()->set_max_iter(
      ilqr_params_.max_iter());
  ilqr_solver_config.mutable_cilqr_config()->set_max_outer_iterations(1);
  ilqr_solver_config.set_enable_cilqr(false);
  // ilqr_solver_config.model_dt = ilqr_params_.delta_s();
  ilqr_solver_config.set_is_debug_mode(true);
  ilqr_solver_config.set_viz_matrix(ilqr_params_.viz_matrix());
  ilqr_solver_config.mutable_ilqr_config()->set_cost_tol(
      ilqr_params_.cost_tol());
  ilqr_solver_config.mutable_ilqr_config()->set_cost_percent_tol(
      ilqr_params_.cost_percent_tol());

  Init(ilqr_solver_config);
  std::cout<<"init success"<<std::endl;
  Update(ilqr_warm_start_data);
  std::cout<<"update success"<<std::endl;

  for(const auto& poly:rb_hPolys){
    auto* rectangle_proto=proto_msg.add_rectangles();
    auto* pt1=rectangle_proto->add_pts();
    pt1->set_x(poly(2,0));
    pt1->set_y(poly(3,0));
    auto* pt2=rectangle_proto->add_pts();
    pt2->set_x(poly(2,1));
    pt2->set_y(poly(3,1));
    auto* pt3=rectangle_proto->add_pts();
    pt3->set_x(poly(2,2));
    pt3->set_y(poly(3,2));
    auto* pt4=rectangle_proto->add_pts();
    pt4->set_x(poly(2,3));
    pt4->set_y(poly(3,3));
  }

  std::cout<<"len of rectangle proto = "<<proto_msg.rectangles_size()<<std::endl;


  const ilqr_solver::ILqr::ILqrSolverInfo* solver_info_ptr=ilqr_core_ptr_->GetSolverInfoPtr();
  for(size_t i=0;i<solver_info_ptr->iteration_info_vec.size();++i){
    if(solver_info_ptr->iteration_info_vec[i].u_vec.size()==0)break;
    if(i==0){
      IlqrWarmStartData* warm_start_proto_data_ptr=proto_msg.mutable_warm_start_data();
      for(const auto& state:solver_info_ptr->iteration_info_vec[i].x_vec){
        IlqrState* temp_state=warm_start_proto_data_ptr->add_states();
        temp_state->set_x(state(StateId::X));
        temp_state->set_y(state(StateId::Y));
        temp_state->set_theta(state(StateId::THETA));
        temp_state->set_kappa(state(StateId::KAPPA));
        temp_state->set_v(state(StateId::V));
      }
      for(const auto& control:solver_info_ptr->iteration_info_vec[i].u_vec){
        IlqrControl* temp_control=warm_start_proto_data_ptr->add_controls();
        temp_control->set_dkappa(control(ControlId::DKAPPA));
        temp_control->set_a(control(ControlId::A));
      }
    }
    else{
      IlqrIterationData* iteration_proto_data_ptr=proto_msg.add_ilqr_iteration_datas();
      for(const auto& state:solver_info_ptr->iteration_info_vec[i].x_vec){
        IlqrState* temp_state=iteration_proto_data_ptr->add_states();
        temp_state->set_x(state(StateId::X));
        temp_state->set_y(state(StateId::Y));
        temp_state->set_theta(state(StateId::THETA));
        temp_state->set_kappa(state(StateId::KAPPA));
        temp_state->set_v(state(StateId::V));
      }
      for(const auto& control:solver_info_ptr->iteration_info_vec[i].u_vec){
        IlqrControl* temp_control=iteration_proto_data_ptr->add_controls();
        temp_control->set_dkappa(control(ControlId::DKAPPA));
        temp_control->set_a(control(ControlId::A));
      }
    }
  }

  // PrintDebugMsg();

}

void IlqrOptimizer::PrintDebugMsg(){
  StateVec result_state_vec=*ilqr_core_ptr_->GetStateResultPtr();
  for(size_t i=0;i<result_state_vec.size();++i){
    std::cout<<"the "<<i+1<<" result point=["<<result_state_vec[i]<<"]"<<std::endl;
  }
  
}

void IlqrOptimizer::Init(const ilqr_solver::SolverConfig& solver_config){
  ilqr_solver_config_ptr_ =
      std::make_unique<ilqr_solver::SolverConfig>(solver_config);
  ilqr_core_ptr_ = std::make_unique<ilqr_solver::ILqr>();
  common_term_calculator_ =
      std::make_unique<IlqrCommonTerm> (ilqr_params_, data_.get());
  ilqr_core_ptr_->Init(planner_model_.get(), ilqr_solver_config_ptr_.get(),
                       common_term_calculator_.get());
  cost_stack_vec_.emplace_back(
      std::make_unique<SmoothCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<ControlCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<TargetVCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<ReflineCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<TerminalCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<CorridorCostTerm>(ilqr_params_, *data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<LateralAccCostTerm>(ilqr_params_, *data_));
  for (const auto &cost_ptr : cost_stack_vec_) {
    ilqr_core_ptr_->AddCost(cost_ptr.get());
  }
  // ilqr_core_ptr_->InitAdvancedInfo();
  common_term_calculator_->SetIlqrCore(ilqr_core_ptr_.get());
  data_->ResizeStepData(Horizon() + 1);
  ResizeAndResetEigenVec(
      x0_, Horizon() + 1,
      ilqr_core_ptr_->GetSolverConfig().ilqr_config().state_size());
  //装填data中的rawpoint
  auto* raw_point_ptr=data_->MutableRawPointVecPtr();
  for(const auto& raw_pt:proto_msg_->raw_points()){
    raw_point_ptr->push_back(raw_pt);
  }
  //装填data中的terminal pt
  if((*data_)(TO_TERMINAL)>0.5){
    *data_->Mutable(TERMINAL_X)=data_->GetRawPointVec().back().x();
    *data_->Mutable(TERMINAL_Y)=data_->GetRawPointVec().back().y();
    *data_->Mutable(TERMINAL_THETA)=data_->GetRawPointVec().back().theta();
  }
  else{   
    *data_->Mutable(TERMINAL_X)=proto_msg_->raw_points(warm_start_end_idx).x();
    *data_->Mutable(TERMINAL_Y)=proto_msg_->raw_points(warm_start_end_idx).y();
    *data_->Mutable(TERMINAL_THETA)=proto_msg_->raw_points(warm_start_end_idx).theta();
  }
}


void IlqrOptimizer::Update(const IlqrWarmStartData& ilqr_warm_start_data){
  UpdateCostConfig();
  UpdateInitState(ilqr_warm_start_data.states(0), &(x0_[0]));
  UpdateWarmStartData(ilqr_warm_start_data);
  Solve();
}

void IlqrOptimizer::Solve() const{
  ilqr_core_ptr_->Solve(x0_[0]);
}

void IlqrOptimizer::UpdateCostConfig(){
  *data_->Mutable(W_SMOOTH_KAPPA)=ilqr_params_.w_smooth_kappa();
  *data_->Mutable(W_CONTROL_DKAPPA)=ilqr_params_.w_control_dkappa();
  *data_->Mutable(W_CONTROL_A)=ilqr_params_.w_control_a();
  *data_->Mutable(W_TARGET_V)=ilqr_params_.w_target_v();
  *data_->Mutable(W_REFLINE)=ilqr_params_.w_refline();
  *data_->Mutable(W_OBS)=ilqr_params_.w_obs();
  *data_->Mutable(W_LATERAL_ACC)=ilqr_params_.w_lateral_acc();
  *data_->Mutable(W_TERMINAL_POS)=ilqr_params_.w_terminal_pos();
  *data_->Mutable(W_TERMINAL_THETA)=ilqr_params_.w_terminal_theta();
  *data_->Mutable(TARGET_V)=ilqr_params_.target_v();
  *data_->Mutable(VEHICLE_LENGTH)=ilqr_params_.vehicle_length();
  *data_->Mutable(VEHICLE_WIDTH)=ilqr_params_.vehicle_width();
  *data_->Mutable(HORIZON)=Horizon();

  std::cout<<"horizon = "<<(*data_)(HORIZON)<<std::endl;
}

void IlqrOptimizer::UpdateInitState(const IlqrState& init_state, State *x ){
  *x << init_state.x(), init_state.y(), init_state.theta(), init_state.kappa(), init_state.v();
  std::cout<<"x[0]="<<(*x)<<std::endl;
}

void IlqrOptimizer::UpdateWarmStartData(const IlqrWarmStartData& ilqr_warm_start_data){
  ControlVec warm_start_u{};
  ResizeAndResetEigenVec(
      warm_start_u, Horizon(),
      ilqr_core_ptr_->GetSolverConfig().ilqr_config().input_size());

  for (int i = 0; i < ilqr_warm_start_data.controls_size(); i++) {
    warm_start_u[i][ControlId::DKAPPA] = ilqr_warm_start_data.controls(i).dkappa();
    warm_start_u[i][ControlId::A] = ilqr_warm_start_data.controls(i).a();
  }

  ilqr_core_ptr_->UpdateWarmStart({warm_start_u});

  State cur_x(static_cast<int>(StateId::STATE_SIZE));
  StateVec temp_x0;
  ResizeAndResetEigenVec(
      temp_x0, Horizon() + 1,
      ilqr_core_ptr_->GetSolverConfig().ilqr_config().state_size());
  for (int i = 0; i < ilqr_warm_start_data.states_size(); i++) {
    cur_x[0] = ilqr_warm_start_data.states(i).x();
    cur_x[1] = ilqr_warm_start_data.states(i).y();
    cur_x[2] = ilqr_warm_start_data.states(i).theta();
    cur_x[3] = ilqr_warm_start_data.states(i).kappa();
    temp_x0[i] = cur_x;
  }
  common_term_calculator_->SetInitState(temp_x0);


}

void IlqrOptimizer::Update_params(){
  auto FixData = [this](FixedDataIndex index) -> double & {
    return *data_->Mutable(index);
  };
  FixData(DELTA_T)=ilqr_params_.delta_t();
}

void IlqrOptimizer::CalcWarmStart(
  const IlqrProtoMsg &proto_msg, const double &dt,
  IlqrWarmStartData* warm_start_data){
  const auto& raw_points=proto_msg.raw_points();
  const auto &ego_state =proto_msg.ego_state();
  auto states = warm_start_data->mutable_states();
  auto controls = warm_start_data->mutable_controls();
  // set init state
  State cur_state(static_cast<int>(StateId::STATE_SIZE));
  cur_state << ego_state.x(), ego_state.y(), ego_state.theta(),
      ego_state.curvature(),ego_state.v();
  std::cout<<"first cur state=["<<cur_state[StateId::X]<<","<<cur_state[StateId::Y]
                      <<","<<cur_state[StateId::THETA]<<","<<cur_state[StateId::KAPPA]<<","<<cur_state[StateId::V]<<"]"<<std::endl;
  Control cur_control(static_cast<int>(ControlId::INPUT_SIZE));
  auto add_init_state = states->Add();
  add_init_state->set_x(ego_state.x());
  add_init_state->set_y(ego_state.y());
  add_init_state->set_theta(ego_state.theta());
  add_init_state->set_kappa(ego_state.curvature());
  add_init_state->set_v(ego_state.v());
  int nearest_point_idx = start_index_;

  constexpr double kLenAhead = 1.5;
  constexpr double kLenAhead_2 = kLenAhead*kLenAhead;
  constexpr int MaxPersuit=100;
  int ii=0;
  while (true) {
    //计算纵向
    constexpr double target_v=5.0;
    constexpr double KAccControl=1.5;
    constexpr double KAccLimit=3.0;
    double acc=-(cur_state[StateId::V]-target_v)*KAccControl;
    acc=clip(acc,-KAccLimit,KAccLimit);
    //计算横向
    double dis_2 = std::numeric_limits<double>::max();
    for (int i = start_index_; i < proto_msg.raw_points_size(); ++i) {
      double temp_dis_2=std::pow(proto_msg.raw_points(i).x()-cur_state[StateId::X],2)
                        +std::pow(proto_msg.raw_points(i).y()-cur_state[StateId::Y],2);
      if (temp_dis_2 < dis_2) {
        dis_2 = temp_dis_2;
        nearest_point_idx = i;
      }
    }
    std::cout<<"nearest_point_idx="<<nearest_point_idx<<std::endl;
    if ((nearest_point_idx >= end_index_)||(ii>MaxPersuit)) {
      warm_start_end_idx=nearest_point_idx-1;
      break;
    }
    // find the lookahead point
    int look_ahead_idx = nearest_point_idx;
    double dis_to_cur_xy_2=std::pow(proto_msg.raw_points(look_ahead_idx).x()-cur_state[StateId::X],2)
                        +std::pow(proto_msg.raw_points(look_ahead_idx).y()-cur_state[StateId::Y],2);
    while ((dis_to_cur_xy_2 < kLenAhead_2) &&
           (look_ahead_idx < (proto_msg.raw_points_size() - 1))) {
      look_ahead_idx++;
      dis_to_cur_xy_2 = std::pow(proto_msg.raw_points(look_ahead_idx).x()-cur_state[StateId::X],2)
                        +std::pow(proto_msg.raw_points(look_ahead_idx).y()-cur_state[StateId::Y],2);
    }
    double angle_cur_to_lookaheadpoint =NormalizeAngle(std::atan2(proto_msg.raw_points(look_ahead_idx).y()-cur_state[StateId::Y]
                                                  ,proto_msg.raw_points(look_ahead_idx).x()-cur_state[StateId::X]));
    double ds=cur_state[StateId::V]*dt+acc*dt*dt*0.5;
    double dis_angle =
        NormalizeAngle(cur_state[StateId::THETA] - angle_cur_to_lookaheadpoint);
    double pure_pursiut_k = -2 * sin(dis_angle) / kLenAhead;
    double dkappa = 2 * (pure_pursiut_k - cur_state[StateId::KAPPA]) / ds;
    cur_control << dkappa, acc;
    planner_model_->LimitControl(cur_state, &cur_control);
    auto next_state =
        planner_model_->UpdateDynamicsWithDt(cur_state, cur_control, dt);
    std::cout<<"cur state=["<<cur_state[StateId::X]<<","<<cur_state[StateId::Y]
                        <<","<<cur_state[StateId::THETA]<<","<<cur_state[StateId::KAPPA]<<","<<cur_state[StateId::V]<<"]"<<std::endl;
    std::cout<<"next state=["<<next_state[StateId::X]<<","<<next_state[StateId::Y]
                              <<","<<next_state[StateId::THETA]<<","<<next_state[StateId::KAPPA]<<","<<next_state[StateId::V]<<"]"<<std::endl;
    // add states
    auto state = states->Add();
    state->set_x(next_state[StateId::X]);
    state->set_y(next_state[StateId::Y]);
    state->set_theta(next_state[StateId::THETA]);
    state->set_kappa(next_state[StateId::KAPPA]);
    state->set_v(next_state[StateId::V]);
    cur_state = next_state;
    // add controls
    auto control = controls->Add();
    control->set_dkappa(cur_control[ControlId::DKAPPA]);
    control->set_a(cur_control[ControlId::A]);
    ii++;
  }
  }


void IlqrOptimizer::MatchStartIndex(){
  double min_dis=99999;
  int match_start_index;
  double ego_x=proto_msg_->ego_state().x();
  double ego_y=proto_msg_->ego_state().y();
  int i=0;
  for(const auto& raw_point:proto_msg_->raw_points()){
    double temp_x=raw_point.x();
    double temp_y=raw_point.y();
    double temp_dis=std::sqrt(std::pow(ego_x-temp_x,2)+std::pow(ego_y-temp_y,2));
    if(temp_dis<min_dis){
      min_dis=temp_dis;
      match_start_index=i;
    }
    i++;
  }
  start_index_=match_start_index;
  end_index_=std::min(start_index_+20,proto_msg_->raw_points_size()-1);
  if(end_index_>=proto_msg_->raw_points_size()-3){
    *data_->Mutable(TO_TERMINAL)=1.0;
  }
}



void IlqrOptimizer::GetRectangleConst(
    std::vector<Eigen::Matrix<double, 4, 4>>* rb_hPolys,const IlqrProtoMsg &proto_msg) {
  static constexpr double kLatExtendStep = 0.1;
  static constexpr double kLonExtendStep = 0.2;
  static constexpr double kCorridorWidthBase = 2.5;
  static constexpr double kCorridorWidthExtendRatio = 0.15;
  static constexpr double kBoxExtendStep = 2.0;
  static constexpr double kLimitBound = 15.0;
  auto cal_lat_extra_extend_length = [](const double corridor_width) -> double {
    return npp::pnc::math_utils::clamp(0.0, 0.5, (corridor_width - kCorridorWidthBase) *
                                      kCorridorWidthExtendRatio);
  };
  size_t state_size=proto_msg.warm_start_data().states_size();
  rb_hPolys->resize(state_size);

  ExtendFinishTable finish_table;
  LineSegment2d extend_line;
  for (int i = 0; i < state_size; ++i) {
    const auto& state = proto_msg.warm_start_data().states(i);
    finish_table.fill(false);
    const double x = state.x();
    const double y = state.y();
    const double yaw = state.theta();
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double x_rear_to_center = cos_yaw * ilqr_params_.d_cr();
    const double y_rear_to_center = sin_yaw * ilqr_params_.d_cr();
    double box_extend_len = 0.0;
    for (box_extend_len = 0.0; box_extend_len < kLimitBound;
         box_extend_len += kBoxExtendStep) {
      Box2d box(
          {x + x_rear_to_center, y + y_rear_to_center}, yaw,
          2.0 * box_extend_len, 2.0 * box_extend_len);
      if (IsCollision(box)) {
        box_extend_len -= kBoxExtendStep;
        break;
      }
    }

    auto cal_extend_len = [&](const ExtendDir dir,
                              const std::array<double, 4>& extend_buffers,
                              LineSegment2d* res) -> void {
      double start_x = 0.0;
      double start_y = 0.0;
      double end_x = 0.0;
      double end_y = 0.0;
      if (dir >= EXTEND_DIR_SIZE) {
        res->Set(x, y, x, y);
        return;
      }
      switch (dir) {
        case LEFT:
          start_x = -cos_yaw * extend_buffers[BOTTOM] -
                    sin_yaw * extend_buffers[LEFT];
          start_y = -sin_yaw * extend_buffers[BOTTOM] +
                    cos_yaw * extend_buffers[LEFT];
          end_x =
              cos_yaw * extend_buffers[TOP] - sin_yaw * extend_buffers[LEFT];
          end_y =
              sin_yaw * extend_buffers[TOP] + cos_yaw * extend_buffers[LEFT];
          break;
        case TOP:
          start_x =
              cos_yaw * extend_buffers[TOP] - sin_yaw * extend_buffers[LEFT];
          start_y =
              sin_yaw * extend_buffers[TOP] + cos_yaw * extend_buffers[LEFT];
          end_x =
              cos_yaw * extend_buffers[TOP] + sin_yaw * extend_buffers[RIGHT];
          end_y =
              sin_yaw * extend_buffers[TOP] - cos_yaw * extend_buffers[RIGHT];
          break;
        case RIGHT:
          start_x =
              cos_yaw * extend_buffers[TOP] + sin_yaw * extend_buffers[RIGHT];
          start_y =
              sin_yaw * extend_buffers[TOP] - cos_yaw * extend_buffers[RIGHT];
          end_x = -cos_yaw * extend_buffers[BOTTOM] +
                  sin_yaw * extend_buffers[RIGHT];
          end_y = -sin_yaw * extend_buffers[BOTTOM] -
                  cos_yaw * extend_buffers[RIGHT];
          break;
        case BOTTOM:
          start_x = -cos_yaw * extend_buffers[BOTTOM] +
                    sin_yaw * extend_buffers[RIGHT];
          start_y = -sin_yaw * extend_buffers[BOTTOM] -
                    cos_yaw * extend_buffers[RIGHT];
          end_x = -cos_yaw * extend_buffers[BOTTOM] -
                  sin_yaw * extend_buffers[LEFT];
          end_y = -sin_yaw * extend_buffers[BOTTOM] +
                  cos_yaw * extend_buffers[LEFT];
          break;
        default:
          break;
      }
      res->Set(x + x_rear_to_center + start_x, y + y_rear_to_center + start_y,
               x + x_rear_to_center + end_x, y + y_rear_to_center + end_y);
    };
    box_extend_len = std::max(0.0, box_extend_len);
    std::array<double, 4> extend_len;
    extend_len.fill(box_extend_len);
    // dcr width length
    auto& poly_info = (*rb_hPolys)[i];
    poly_info.col(0).head<2>() << -sin_yaw, cos_yaw;
    poly_info.col(1).head<2>() << cos_yaw, sin_yaw;
    poly_info.col(2).head<2>() << sin_yaw, -cos_yaw;
    poly_info.col(3).head<2>() << -cos_yaw, -sin_yaw;
    while (!IsExentFinish(finish_table)) {
      for (int dir = LEFT; dir < EXTEND_DIR_SIZE; ++dir) {
        if (finish_table[dir]) {
          continue;
        }
        const double step_length =
            ((dir == LEFT || dir == RIGHT) ? kLatExtendStep : kLonExtendStep);
        extend_len[dir] += step_length;
        if (extend_len[dir] >= kLimitBound) {
          extend_len[dir] -= step_length;
          finish_table[dir] = true;
          break;
        }
        cal_extend_len(static_cast<ExtendDir>(dir), extend_len, &extend_line);
        finish_table[dir] = IsCollision(extend_line);
        if (finish_table[dir]) {
          extend_len[dir] -= step_length;
        }
      }
    }

    for (int dir = 0; dir < EXTEND_DIR_SIZE; ++dir) {
      cal_extend_len(static_cast<ExtendDir>(dir), extend_len, &extend_line);
      const auto& indices = col_indices[dir];
      poly_info.col(indices.first).tail<2>() << extend_line.start().x(),
          extend_line.start().y();
    }


  }
}

bool IlqrOptimizer::IsExentFinish(const ExtendFinishTable table) {
  return table[LEFT] && table[TOP] && table[RIGHT] && table[BOTTOM];
}

void IlqrOptimizer::AddProtoObb(){
  for(const auto& polygen:proto_msg_->polygens()){
    std::vector<Eigen::Vector2d> points;
    for(const auto& pt:polygen.pts()){
      Eigen::Vector2d vec(pt.x(), pt.y());
      points.push_back(vec);
    }
    Obbox temp_obbox{points};
    obb_list.emplace_back(temp_obbox);
  }

}

bool IlqrOptimizer::IsCollision(const Box2d& box) const {

  for (const auto& obb : obb_list) {
    if (obb.IsCollision(box)) {
      return true;
    }
  }
  return false;
}

bool IlqrOptimizer::IsCollision(const LineSegment2d& line) const {
  for (const auto& obb : obb_list) {
    if (obb.IsCollision(line)) {
      return true;
    }
  }
  return false;
}



//IlqrModel
State IlqrPlannerModel::UpdateDynamicsOneStep(const State &x, const Control &u,
                            const size_t &step) const{
  const double delta_t = data_(DELTA_T);
  return UpdateDynamicsWithDt(x, u, delta_t);   
                            }

void IlqrPlannerModel::GetDynamicsDerivatives(const State &x, const Control &u,
                            const size_t &step, FxMT *const f_x,
                            FuMT *const f_u) const{
  const double cur_x = x[StateId::X];
  const double cur_y = x[StateId::Y];
  const double cur_theta = x[StateId::THETA];
  const double cur_kappa = x[StateId::KAPPA];
  const double cur_v=x[StateId::V];
  const double dt = data_(DELTA_T);
  const double dt_6=dt/6;
  const double ds = cur_v*dt+u[ControlId::A]*dt*dt*0.5;
  const double half_ds = 0.5 * ds;
  const double ds_6 = ds / 6.0;
  const double dkappa = u[ControlId::DKAPPA];

  // calc RK4 for derivatives
  std::vector<double> rk4_k(RK4_SIZE, 0.0);
  CalcRK4(cur_theta, cur_kappa, dkappa, ds, rk4_k);
  const double dxk2dv=-rk4_k[k2_y]*(cur_kappa*dt*0.5+0.25*dkappa*ds*dt);
  const double dxk4dv=-rk4_k[k4_y]*(cur_kappa*dt+dkappa*ds*dt);
  const double dxk2da=-rk4_k[k2_y]*(0.25*cur_kappa*dt*dt+0.125*dkappa*ds*dt*dt);
  const double dxk4da=-rk4_k[k4_y]*(0.5*cur_kappa*dt*dt+0.5*dkappa*ds*dt*dt);
  const double dyk2dv=rk4_k[k2_x]*(cur_kappa*dt*0.5+0.25*dkappa*ds*dt);
  const double dyk4dv=rk4_k[k4_x]*(cur_kappa*dt+dkappa*ds*dt);
  const double dyk2da=rk4_k[k2_x]*(0.25*cur_kappa*dt*dt+0.125*dkappa*ds*dt*dt);
  const double dyk4da=rk4_k[k4_x]*(0.5*cur_kappa*dt*dt+0.5*dkappa*ds*dt*dt);

  const double dxk2dkappa = -rk4_k[k2_y] * half_ds;
  const double dxk4dkappa = -rk4_k[k4_y] * ds;
  const double dyk2dkappa = rk4_k[k2_x] * half_ds;
  const double dyk4dkappa = rk4_k[k4_x] * ds;

  const double dxk1dtheta = -rk4_k[k1_y];
  const double dxk2dtheta = -rk4_k[k2_y];
  const double dxk4dtheta = -rk4_k[k4_y];
  const double dyk1dtheta = rk4_k[k1_x];
  const double dyk2dtheta = rk4_k[k2_x];
  const double dyk4dtheta = rk4_k[k4_x];  
  // compute Jacobian
  f_x->setIdentity();
  (*f_x)(X, THETA) = ds_6 * (dxk1dtheta + 4 * dxk2dtheta + dxk4dtheta);
  (*f_x)(X, KAPPA) = ds_6 * (4 * dxk2dkappa + dxk4dkappa);
  (*f_x)(Y, THETA) = ds_6 * (dyk1dtheta + 4 * dyk2dtheta + dyk4dtheta);
  (*f_x)(Y, KAPPA) = ds_6 * (4 * dyk2dkappa + dyk4dkappa);
  (*f_x)(X, V)=dt_6*(rk4_k[k1_x]+4*rk4_k[k2_x]+rk4_k[k4_x])+ds_6*(4*dxk2dv+dxk4dv);
  (*f_x)(Y, V)=dt_6*(rk4_k[k1_y]+4*rk4_k[k2_y]+rk4_k[k4_y])+ds_6*(4*dyk2dv+dyk4dv);
  (*f_x)(KAPPA, V)=dkappa*dt;
  (*f_x)(THETA, KAPPA)=ds;
  (*f_x)(THETA, V)=cur_kappa*dt+dkappa*ds*dt;

  (*f_u)(X, DKAPPA) = ds_6*(-0.5*rk4_k[k2_y]*ds*ds-0.5*rk4_k[k4_y]*ds*ds);
  (*f_u)(Y, DKAPPA) = ds_6*(0.5*rk4_k[k2_x]*ds*ds+0.5*rk4_k[k4_x]*ds*ds);
  (*f_u)(X, A) =(1/12)*dt*dt*(rk4_k[k1_x]+4*rk4_k[k2_x]+rk4_k[k4_x])+ds_6*(4*dxk2da+dxk4da);
  (*f_u)(Y, A) =(1/12)*dt*dt*(rk4_k[k1_y]+4*rk4_k[k2_y]+rk4_k[k4_y])+ds_6*(4*dyk2da+dyk4da);
  (*f_u)(V, A) =dt;
  (*f_u)(KAPPA, DKAPPA) =ds;
  (*f_u)(KAPPA, A) =0.5*dkappa*dt*dt;
  (*f_u)(THETA, DKAPPA) =0.5*ds*ds;
  (*f_u)(THETA, A) =0.5*cur_kappa*dt*dt+0.5*dkappa*ds*dt*dt; 
}

State IlqrPlannerModel::UpdateDynamicsWithDt(const State &x, const Control &u,
                                  const double &dt){
  const double cur_x = x[StateId::X];
  const double cur_y = x[StateId::Y];
  const double cur_theta = x[StateId::THETA];
  const double cur_kappa = x[StateId::KAPPA];
  const double cur_v=x[StateId::V];

  double ds=cur_v*dt+u[ControlId::A]*dt*dt*0.5;
  // std::cout<<"model cal ds="<<ds<<std::endl;
  double v_next=cur_v+u[ControlId::A]*dt;
  double kappa_next = cur_kappa + u[ControlId::DKAPPA] * ds;
  double theta_next =
      cur_theta + cur_kappa * ds + 0.5 * u[ControlId::DKAPPA] * ds * ds;

  std::vector<double> rk4_k(RK4_SIZE, 0.0);
  CalcRK4(cur_theta, cur_kappa, u[ControlId::DKAPPA], ds, rk4_k);
  // std::cout<<"model cal xk1="<<rk4_k[k1_x]<<std::endl;
  double x_next =
      cur_x + ds / 6 * (rk4_k[k1_x] + 4 * rk4_k[k2_x] + rk4_k[k4_x]);
  double y_next =
      cur_y + ds / 6 * (rk4_k[k1_y] + 4 * rk4_k[k2_y] + rk4_k[k4_y]);

  State output_state{};
  output_state.resize(STATE_SIZE);
  output_state << x_next, y_next, theta_next, kappa_next,v_next;
  // std::cout<<"model cal x="<<x_next<<std::endl;
  // std::cout<<"model cal state=["<<output_state[StateId::X]<<","<<output_state[StateId::Y]
  //                     <<","<<output_state[StateId::THETA]<<","<<output_state[StateId::KAPPA]<<","<<output_state[StateId::V]<<"]"<<std::endl;


  return output_state;
}

void IlqrPlannerModel::LimitControl(const State &x, Control *const u) const{

}

void IlqrPlannerModel::CalcRK4(const double &theta, const double &kappa,
                    const double &dkappa, const double &ds,
                    std::vector<double> &k){
  const double half_ds = ds / 2;
  k[k1_x] = std::cos(theta);
  k[k1_y] = std::sin(theta);
  k[k2_x] =
      std::cos(theta + kappa * half_ds + 0.5 * dkappa * half_ds * half_ds);
  k[k2_y] =
      std::sin(theta + kappa * half_ds + 0.5 * dkappa * half_ds * half_ds);
  k[k4_x] = std::cos(theta + kappa * ds + 0.5 * dkappa * ds * ds);
  k[k4_y] = std::sin(theta + kappa * ds + 0.5 * dkappa * ds * ds);
}


//common term


void IlqrCommonTerm::ComputeCommonTerms(const State &x, const Control & /*u*/,
                        const size_t step) {
  if(ilqr_core_ptr_->GetSolverInfoPtr()->iter_count<=1){
    double min_dis_2=999999;
    double cloest_x,cloest_y;
    for(const auto& raw_pt:data_->GetRawPointVec()){
      double temp_dis_2=std::pow(x(StateId::X)-raw_pt.x(),2)+std::pow(x(StateId::Y)-raw_pt.y(),2);
      if(temp_dis_2<min_dis_2){
        min_dis_2=temp_dis_2;
        cloest_x=raw_pt.x();
        cloest_y=raw_pt.y();
      }
    }
    *(data_->Mutable(X_REF,step))=cloest_x;
    *(data_->Mutable(Y_REF,step))=cloest_y;
  }

}


  //SmoothCost
double SmoothCostTerm::GetCost(const State & x, const Control & /*u*/,
                        const size_t /*step*/) const {
  double cost={0.0};
  cost+=0.5*data_(W_SMOOTH_KAPPA)*x[StateId::KAPPA]*x[StateId::KAPPA];
  return cost;
}
void SmoothCostTerm::GetGradientHessian(const State & x, const Control & /*u*/,
                                const size_t /*step*/, LxMT *const lx,
                                LuMT *const /*lu*/, LxxMT *const lxx,
                                LxuMT *const /*lxu*/,
                                LuuMT *const /*luu*/) const {
  (*lx)(3)=data_(W_SMOOTH_KAPPA)*x[StateId::KAPPA];    
  (*lxx)(3,3)=data_(W_SMOOTH_KAPPA);               
}



  //ControlCost
double ControlCostTerm::GetCost(const State & /*x*/, const Control &u,
              const size_t step) const {
  double cost={0.0};

  cost += 0.5*data_(W_CONTROL_A)*u[ControlId::A]*u[ControlId::A]
          +0.5*data_(W_CONTROL_DKAPPA)*u[ControlId::DKAPPA]*u[ControlId::DKAPPA];
  return cost;
                
}
void ControlCostTerm::GetGradientHessian(const State & /*x*/, const Control &u,
                        const size_t step, LxMT *const /*lx*/, LuMT *const lu,
                        LxxMT *const /*lxx*/, LxuMT *const /*lxu*/,
                        LuuMT *const luu) const {


  (*lu)(0) += data_(W_CONTROL_DKAPPA) * u[ControlId::DKAPPA];
  (*lu)(1) += data_(W_CONTROL_A)*u[ControlId::A] ;
  (*luu)(0, 0) += data_(W_CONTROL_DKAPPA);
  (*luu)(1, 1) += data_(W_CONTROL_A);
                          
}


//target v cost
double TargetVCostTerm::GetCost(const State & x, const Control & /*u*/,
                        const size_t /*step*/) const {
  double cost{0.0};
  cost+=0.5*data_(W_TARGET_V)*(x[StateId::V]-data_(TARGET_V))*(x[StateId::V]-data_(TARGET_V));

  return cost;
}
void TargetVCostTerm::GetGradientHessian(const State & x, const Control & /*u*/,
                                const size_t /*step*/, LxMT *const lx,
                                LuMT *const /*lu*/, LxxMT *const lxx,
                                LxuMT *const /*lxu*/,
                                LuuMT *const /*luu*/) const {
  (*lx)(4)=data_(W_TARGET_V)*(x[StateId::V]-data_(TARGET_V));
  (*lxx)(4,4)=data_(W_TARGET_V);

}

double ReflineCostTerm::GetCost(const State & x, const Control &/*u*/,
                const size_t step) const{
  double cost=0.0;
  cost+=data_(W_REFLINE)*(std::pow(x(StateId::X)-data_(X_REF,step),2)+std::pow(x(StateId::Y)-data_(Y_REF,step),2));
  return cost;

}
void ReflineCostTerm::GetGradientHessian(const State & x, const Control &/*u*/,
                        const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                        LxxMT *const lxx, LxuMT *const /*lxu*/,
                        LuuMT *const /*luu*/) const{
  (*lx)(0)=2*data_(W_REFLINE)*(x(StateId::X)-data_(X_REF,step));
  (*lx)(1)=2*data_(W_REFLINE)*(x(StateId::Y)-data_(Y_REF,step));
  (*lxx)(0,0)=2*data_(W_REFLINE);
  (*lxx)(1,1)=2*data_(W_REFLINE);
}

void CorridorCostTerm::GetVehicleCornerPt(std::vector<State>& vehicle_corner_pts,std::vector<Eigen::Vector2d>& raw_corner,const State & x) const{
  double ego_x=x[StateId::X];
  double ego_y=x[StateId::Y];
  Eigen::Matrix<double,2,2> rotate_matrix;
  rotate_matrix<<std::cos(x[StateId::THETA]),-std::sin(x[StateId::THETA])
                          ,std::sin(x[StateId::THETA]),std::cos(x[StateId::THETA]);
  Eigen::Vector2d ego_pos;
  ego_pos<<ego_x,ego_y;

  Eigen::Vector2d corner1_vec;
  corner1_vec<<-data_(VEHICLE_LENGTH)/2,data_(VEHICLE_WIDTH)/2;
  Eigen::Vector2d corner2_vec;
  corner2_vec<<data_(VEHICLE_LENGTH)/2,data_(VEHICLE_WIDTH)/2;
  Eigen::Vector2d corner3_vec;
  corner3_vec<<data_(VEHICLE_LENGTH)/2,-data_(VEHICLE_WIDTH)/2;
  Eigen::Vector2d corner4_vec;
  corner4_vec<<-data_(VEHICLE_LENGTH)/2,-data_(VEHICLE_WIDTH)/2;
  raw_corner.emplace_back(corner1_vec);
  raw_corner.emplace_back(corner2_vec);
  raw_corner.emplace_back(corner3_vec);
  raw_corner.emplace_back(corner4_vec);

  Eigen::Vector2d corner1_global=rotate_matrix*corner1_vec+ego_pos;
  Eigen::Vector2d corner2_global=rotate_matrix*corner2_vec+ego_pos;
  Eigen::Vector2d corner3_global=rotate_matrix*corner3_vec+ego_pos;
  Eigen::Vector2d corner4_global=rotate_matrix*corner4_vec+ego_pos;

  State corner1(int(StateId::STATE_SIZE));
  corner1(StateId::X)=corner1_global(0);
  corner1(StateId::Y)=corner1_global(1);
  vehicle_corner_pts.emplace_back(corner1);
  State corner2(int(StateId::STATE_SIZE));
  corner2(StateId::X)=corner2_global(0);
  corner2(StateId::Y)=corner2_global(1);
  vehicle_corner_pts.emplace_back(corner2);
  State corner3(int(StateId::STATE_SIZE));
  corner3(StateId::X)=corner3_global(0);
  corner3(StateId::Y)=corner3_global(1);
  vehicle_corner_pts.emplace_back(corner3);
  State corner4(int(StateId::STATE_SIZE));
  corner4(StateId::X)=corner4_global(0);
  corner4(StateId::Y)=corner4_global(1);
  vehicle_corner_pts.emplace_back(corner4);
  return;

}

double CorridorCostTerm::GetCost(const State & x, const Control &/*u*/,
                const size_t step) const {
  double cost=0.0;
  std::vector<State> vehicle_corner_pts;
  std::vector<Eigen::Vector2d> raw_corner;
  GetVehicleCornerPt(vehicle_corner_pts,raw_corner,x);
  const auto& rb_hpoly=data_.GetRBHPolys()[step];
  for(const auto& corner_pt:vehicle_corner_pts){
    for(int i=0;i<4;++i){
      Vec2d outer_norm=Vec2d(rb_hpoly(0,i),rb_hpoly(1,i));
      Vec2d corridor2corner_vec=Vec2d(corner_pt(StateId::X)-rb_hpoly(2,i),corner_pt(StateId::Y)-rb_hpoly(3,i));
      double innerprod=outer_norm.InnerProd(corridor2corner_vec);
      if(innerprod>0){
        cost+=data_(W_OBS)*innerprod*innerprod;
      }
    }
  }
  return cost;
}

//d_corner_x_d_x=1
void CorridorCostTerm::GetGradientHessian(const State & x, const Control &/*u*/,
                        const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                        LxxMT *const lxx, LxuMT *const /*lxu*/,
                        LuuMT *const /*luu*/) const{
  std::vector<State> vehicle_corner_pts;
  std::vector<Eigen::Vector2d> raw_corner;
  GetVehicleCornerPt(vehicle_corner_pts,raw_corner,x);
  const auto& rb_hpoly=data_.GetRBHPolys()[step];
  for(int k=0;k<vehicle_corner_pts.size();k++){
    for(int i=0;i<4;++i){
      Vec2d outer_norm=Vec2d(rb_hpoly(0,i),rb_hpoly(1,i));
      Vec2d corridor2corner_vec=Vec2d(vehicle_corner_pts[k](StateId::X)-rb_hpoly(2,i),vehicle_corner_pts[k](StateId::Y)-rb_hpoly(3,i));
      double innerprod=outer_norm.InnerProd(corridor2corner_vec);
      if(innerprod>0){
        double sin_theta=std::sin(x[StateId::THETA]);
        double cos_theta=std::cos(x[StateId::THETA]);
        double dinnerprod_dtheta=-sin_theta*raw_corner[k](0)*outer_norm.x()
                                -cos_theta*raw_corner[k](1)*outer_norm.x()
                                +cos_theta*raw_corner[k](0)*outer_norm.y()
                                -sin_theta*raw_corner[k](1)*outer_norm.y();
        double dinnerprod_dtheta_dtheta=-cos_theta*raw_corner[k](0)*outer_norm.x()
                                +sin_theta*raw_corner[k](1)*outer_norm.x()
                                -sin_theta*raw_corner[k](0)*outer_norm.y()
                                -cos_theta*raw_corner[k](1)*outer_norm.y();
        (*lx)(0)+=2*innerprod*data_(W_OBS)*(outer_norm.x());
        (*lx)(1)+=2*innerprod*data_(W_OBS)*(outer_norm.y());
        (*lx)(2)+=2*innerprod*data_(W_OBS)*dinnerprod_dtheta;
        (*lxx)(2,2)+=2*data_(W_OBS)*(dinnerprod_dtheta*dinnerprod_dtheta+innerprod*dinnerprod_dtheta_dtheta);
      }
    }
  }
}


double TerminalCostTerm::GetCost(const State & x, const Control &/*u*/,
                const size_t step) const {
  // if(data_(TO_TERMINAL)<0.5){
  //   return 0.0;
  // }
  if(double(step)<data_(HORIZON)){
    return 0.0;
  }
  std::cout<<"x= "<<x(StateId::X)<<"  y="<<x(StateId::Y)<<"   theta="<<x(StateId::THETA)<<std::endl;
  std::cout<<"terminal x= "<<data_(TERMINAL_X)<<"  ternimal y="<<data_(TERMINAL_Y)<<"  terminal theta="<<data_(TERMINAL_THETA)<<std::endl;
  std::cout<<"delta theta="<<NormalizeAngle(x(StateId::THETA)-data_(TERMINAL_THETA))<<std::endl;
  double cost=0.0;
  cost+=data_(W_TERMINAL_POS)*(std::pow(x(StateId::X)-data_(TERMINAL_X),2)+std::pow(x(StateId::Y)-data_(TERMINAL_Y),2));
  cost+=data_(W_TERMINAL_THETA)*std::pow(NormalizeAngle(x(StateId::THETA)-data_(TERMINAL_THETA)),2);
  return cost;

}
void TerminalCostTerm::GetGradientHessian(const State & x, const Control &/*u*/,
                        const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                        LxxMT *const lxx, LxuMT *const /*lxu*/,
                        LuuMT *const /*luu*/) const {
  // if(data_(TO_TERMINAL)<0.5){
  //   return;
  // }
  if(double(step)<data_(HORIZON)){
    return;
  }
  (*lx)(0)+=2*data_(W_TERMINAL_POS)*(x(StateId::X)-data_(TERMINAL_X));
  (*lx)(1)+=2*data_(W_TERMINAL_POS)*(x(StateId::Y)-data_(TERMINAL_Y));
  (*lx)(2)+=2*data_(W_TERMINAL_THETA)*(NormalizeAngle(x(StateId::THETA)-data_(TERMINAL_THETA)));
  (*lxx)(0,0)+=2*data_(W_TERMINAL_POS);
  (*lxx)(1,1)+=2*data_(W_TERMINAL_POS);
  (*lxx)(2,2)+=2*data_(W_TERMINAL_THETA);
}


double LateralAccCostTerm::GetCost(const State & x, const Control &/*u*/,
                const size_t step) const {
  double cost=0.0;
  cost+=data_(W_LATERAL_ACC)*x[StateId::V]*x[StateId::V]*x[StateId::KAPPA]*x[StateId::KAPPA];
  return cost;
}
void LateralAccCostTerm::GetGradientHessian(const State & x, const Control &/*u*/,
                        const size_t step, LxMT *const lx, LuMT *const /*lu*/,
                        LxxMT *const lxx, LxuMT *const /*lxu*/,
                        LuuMT *const /*luu*/) const {
  (*lx)(4)+=2*data_(W_LATERAL_ACC)*x(StateId::V)*x(StateId::KAPPA)*x(StateId::KAPPA);
  (*lx)(3)+=2*data_(W_LATERAL_ACC)*x(StateId::KAPPA)*x(StateId::V)*x(StateId::V);

  (*lxx)(4,3)+=4*data_(W_LATERAL_ACC)*x(StateId::V)*x(StateId::KAPPA);
  (*lxx)(4,4)+=2*data_(W_LATERAL_ACC)*x(StateId::KAPPA)*x(StateId::KAPPA);
  (*lxx)(3,3)+=2*data_(W_LATERAL_ACC)*x(StateId::V)*x(StateId::V);
  (*lxx)(3,4)+=4*data_(W_LATERAL_ACC)*x(StateId::V)*x(StateId::KAPPA);

}