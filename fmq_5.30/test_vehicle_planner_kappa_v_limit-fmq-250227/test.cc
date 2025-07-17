#include "speed_planner/piecewise_jerk_speed_nonlinear_optimizer.h"
#include <iostream>
#include <cmath>
#include "speed_planner_config.pb.h"

using apollo::common::PathPoint;

double Distance(const PathPoint& p1, const PathPoint& p2) {
    return std::hypot(p1.x() - p2.x(), p1.y() - p2.y());
}

// 计算三个点组成的夹角的余弦值
double CosineOfAngle(const PathPoint& p1, const PathPoint& p2, const PathPoint& p3) {
    double a = Distance(p1, p2);
    double b = Distance(p2, p3);
    double c = Distance(p1, p3);
    return (a * a + b * b - c * c) / (2 * a * b);
}

// 简化版的计算曲率函数
double CalculateKappa(const PathPoint& p1, const PathPoint& p2, const PathPoint& p3) {
    double cosine_angle = CosineOfAngle(p1, p2, p3);
    double angle = std::acos(cosine_angle);
    double curvature = 2 * angle / Distance(p1, p3); // 这是一个基本的曲率计算公式
    return curvature;
}

void CalKappas(const std::vector<apollo::common::PathPoint>& path_points,std::vector<double>& kappa
                ,std::vector<double>& dkappa,std::vector<double>& ddkappa){
    kappa.push_back(0);
    dkappa.push_back(0);
    dkappa.push_back(0);
    ddkappa.push_back(0);
    ddkappa.push_back(0);
    ddkappa.push_back(0);
    for(size_t i=1;i<path_points.size()-1;++i){
        double k=CalculateKappa(path_points[i-1],path_points[i],path_points[i+1]);
        kappa.push_back(k);
        if(i==1)continue;
        double ds=std::hypot(path_points[i].x()-path_points[i-1].x(),path_points[i].y()-path_points[i-1].y());
        double dk=k-kappa.back();
        double dkds=dk/ds;
        dkappa.push_back(dkds);
        if(i==2)continue;
        double ddk=dkds-dkappa.back();
        double ddkds=ddk/ds;
        ddkappa.push_back(ddkds);
    }
    kappa.push_back(0);
    dkappa.push_back(0);
    ddkappa.push_back(0);   
}

int main(){
    std::cout<<"start main fun"<<std::endl;

    SpeedPlannerConfig config;
    config.set_acc_weight(1);
    config.set_jerk_weight(1);
    config.set_lat_acc_weight(0.01);
    config.set_ref_s_weight(1);
    config.set_ref_v_weight(15);
    config.set_s_potential_weight(1);
    config.set_use_warm_start(true);
    config.set_target_time(3);
    config.set_target_s(10.87);
    apollo::planning::PiecewiseJerkSpeedNonlinearOptimizer nlp_optmizer(config);

    std::vector<apollo::common::PathPoint> path_points;
    size_t len_path=30;
    apollo::common::PathPoint pt_last{};
    double s_now=0.0;
    pt_last.set_x(0);
    pt_last.set_y(0);
    pt_last.set_theta(0.01);
    pt_last.set_s(s_now);
    path_points.push_back(pt_last);
    for(size_t i=1;i<len_path;++i){
        apollo::common::PathPoint pt{};
        pt.set_x(i);
        pt.set_y(i*i*0.1);
        pt.set_theta(std::atan2(pt.y()-pt_last.y(),pt.x()-pt_last.x()));
        s_now+=std::pow(std::pow(pt.x()-pt_last.x(),2)+std::pow(pt.y()-pt_last.y(),2),0.5);
        pt.set_s(s_now);
        pt_last=pt;
        path_points.push_back(pt);
    }
    std::cout<<"start s="<<path_points.front().s();
    std::cout<<"terminal s="<<path_points.back().s()<<std::endl;
    std::vector<double> kappa{};
    std::vector<double> dkappa{};
    std::vector<double> ddkappa{};
    CalKappas(path_points,kappa,dkappa,ddkappa);
    for(size_t i=0;i<path_points.size();++i){
        path_points[i].set_kappa(kappa[i]);
        path_points[i].set_dkappa(dkappa[i]);
        path_points[i].set_ddkappa(ddkappa[i]);
    }

    apollo::planning::DiscretizedPath discretize_path(path_points);
    apollo::planning::PathData path_data;
    path_data.SetDiscretizedPath(discretize_path);

    apollo::common::TrajectoryPoint init_path_pt{};
    auto* init_pt_ptr=init_path_pt.mutable_path_point();
    *init_pt_ptr=path_points[0];
    init_path_pt.set_v(0);
    init_path_pt.set_a(0);
    init_path_pt.set_da(0);
    std::cout<<"start nlp process"<<std::endl;
    nlp_optmizer.Process(path_data,init_path_pt);
    std::cout<<"end nlp process"<<std::endl;

}
