#include <Eigen/Dense>
#include <iostream>
#include "planning_core/inc/ilqr_planner.h"

int main(){
    Eigen::MatrixXd matD;
    matD = Eigen::MatrixXd::Random(4, 4);
    std::cout<<"matD:"<<matD<<std::endl;
    IlqrParams params{};
    params.set_delta_t(0.2);
    params.set_cost_tol(1);
    params.set_cost_percent_tol(0.0001);
    params.set_w_control_a(1);
    params.set_w_control_dkappa(1);
    params.set_w_smooth_kappa(1);
    params.set_max_iter(10);
    params.set_target_v(5);
    params.set_w_target_v(1);

    IlqrOptimizer ilqr_optimizer(params);

    //生成简单的优化前路径
    int step_raw_path=20;
    IlqrProtoMsg proto_msg{};
    for(int i=0;i<step_raw_path;++i){
        double temp_x=i;
        double temp_y=0.1*i*i;
        RawPoint* added_point=proto_msg.add_raw_points();
        added_point->set_x(temp_x);
        added_point->set_y(temp_y);
    }

    //自车状态
    common::State* ego_state=proto_msg.mutable_ego_state();
    ego_state->set_x(0);
    ego_state->set_y(0);
    ego_state->set_theta(0.1);
 

    //optimizer.process
    ilqr_optimizer.Process(proto_msg);
}