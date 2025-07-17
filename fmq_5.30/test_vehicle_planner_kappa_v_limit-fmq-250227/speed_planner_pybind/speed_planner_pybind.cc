#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <iostream>
#include "speed_planner/piecewise_jerk_speed_nonlinear_optimizer.cc"
#include "speed_planner_msg.pb.h"
namespace py = pybind11; // 定义命名空间别名

double NormalizeAngle(double angle){
    while(angle>M_PI){
        angle-=2*M_PI;
    }
    while(angle<-M_PI){
        angle+=2*M_PI;
    }
    return angle;
}


double Distance(const apollo::common::PathPoint& p1, const apollo::common::PathPoint& p2) {
    return std::hypot(p1.x() - p2.x(), p1.y() - p2.y());
}

// 计算三个点组成的夹角的余弦值
double CosineOfAngle(const apollo::common::PathPoint& p1, const apollo::common::PathPoint& p2, const apollo::common::PathPoint& p3) {
    double a = Distance(p1, p2);
    double b = Distance(p2, p3);
    double c = Distance(p1, p3);
    return (a * a + b * b - c * c) / (2 * a * b);
}

// 简化版的计算曲率函数
double CalculateKappa(const apollo::common::PathPoint& p1, const apollo::common::PathPoint& p2, const apollo::common::PathPoint& p3) {
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


class TestPybind{
public:
    TestPybind()=delete;
    TestPybind(double k){
        k_=k;
        std::cout<<"k= "<<k<<std::endl;
    }
    double k_;
};

apollo::planning::PathData GeneratePathData(const SpeedPlannerProtoMsg proto_msg){
    std::vector<apollo::common::PathPoint> path_points;
    for(const auto& pt:proto_msg.opt_path_points()){
        apollo::common::PathPoint temp_pt{};
        temp_pt.set_x(pt.x());
        temp_pt.set_y(pt.y());
        temp_pt.set_theta(pt.theta());
        temp_pt.set_s(pt.s());
        path_points.push_back(temp_pt);
    }
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
    return path_data;

}

void SetSpeedLinear(SpeedPlannerProtoMsg& proto_msg,const std::vector<double>& res_distance
                    ,const std::vector<double>& res_v,const std::vector<double>& res_a){
    //优化出的速度点隔n个取一个
    for(size_t i=0;i<res_distance.size();++i){
        if((i%1)!=0){
            continue;
        }
        double x,y,theta,s,v,a,kappa;
        bool is_in_path=false;
        for(size_t j=0;j<proto_msg.opt_path_points_size()-1;j++){
            //优化后的第一个s点可能是负数，所以加了这个逻辑
            if(res_distance[i]<0){
                x=proto_msg.opt_path_points(0).x();
                y=proto_msg.opt_path_points(0).y();
                theta=proto_msg.opt_path_points(0).theta();
                kappa=proto_msg.opt_path_points(0).kappa();
                s=0.0;
                v=res_v[0];
                a=res_a[0];
                is_in_path=true;
                break;
            }
            if((res_distance[i]>=proto_msg.opt_path_points(j).s())&&(res_distance[i]<=proto_msg.opt_path_points(j+1).s())){
                double ratio=(res_distance[i]-proto_msg.opt_path_points(j).s())/(proto_msg.opt_path_points(j+1).s()-proto_msg.opt_path_points(j).s());
                x=proto_msg.opt_path_points(j).x()+ratio*(proto_msg.opt_path_points(j+1).x()-proto_msg.opt_path_points(j).x());
                y=proto_msg.opt_path_points(j).y()+ratio*(proto_msg.opt_path_points(j+1).y()-proto_msg.opt_path_points(j).y());
                kappa=proto_msg.opt_path_points(j).kappa()+ratio*(proto_msg.opt_path_points(j+1).kappa()-proto_msg.opt_path_points(j).kappa());
                theta=proto_msg.opt_path_points(j).theta()+ratio*NormalizeAngle(proto_msg.opt_path_points(j+1).theta()-proto_msg.opt_path_points(j).theta());
                s=res_distance[i];
                v=res_v[i];
                a=res_a[i];
                is_in_path=true;
                break;
            }
        }
        if(!is_in_path)break;
        auto* traj_point_ptr=proto_msg.add_traj_points();
        traj_point_ptr->set_s(s);
        traj_point_ptr->set_v(v);
        traj_point_ptr->set_a(a);
        traj_point_ptr->set_x(x);
        traj_point_ptr->set_y(y);
        traj_point_ptr->set_kappa(kappa);
        traj_point_ptr->set_theta(theta); 

    }
    return;
}

static py::bytes RunSpeedPlanner(const std::string &speed_planner_proto_msg,
                                     const std::string &config) {
  SpeedPlannerProtoMsg proto_msg{};
  SpeedPlannerConfig params{};
  proto_msg.ParseFromString(speed_planner_proto_msg);
  params.ParseFromString(config);

  apollo::planning::PiecewiseJerkSpeedNonlinearOptimizer nlp_optmizer(params);

  const auto& path_data=GeneratePathData(proto_msg);
  apollo::common::TrajectoryPoint init_path_pt{};
  auto* init_pt_ptr=init_path_pt.mutable_path_point();
  *init_pt_ptr=path_data.discretized_path()[0];
  init_path_pt.set_v(proto_msg.init_v());
  init_path_pt.set_a(proto_msg.init_a());
  init_path_pt.set_da(proto_msg.init_jerk());
  std::cout<<"start nlp process"<<std::endl;
  nlp_optmizer.Process(path_data,init_path_pt);
  std::cout<<"end nlp process"<<std::endl;

  const std::vector<double>& res_distance=nlp_optmizer.GetResDistance();
  const std::vector<double>& res_v=nlp_optmizer.GetResVelocity();
  const std::vector<double>& res_a=nlp_optmizer.GetResAcceleration();

  SetSpeedLinear(proto_msg,res_distance,res_v,res_a);

  return py::bytes(proto_msg.SerializeAsString());

}

PYBIND11_MODULE(speed_planner_pybind, mod) {
    py::class_<TestPybind>(mod,"TestPybind")
        .def(py::init<double>()) 
        .def_readwrite("k", &TestPybind::k_);
    mod.def("RunSpeedPlanner", &RunSpeedPlanner, "RunSpeedPlanner");
}

