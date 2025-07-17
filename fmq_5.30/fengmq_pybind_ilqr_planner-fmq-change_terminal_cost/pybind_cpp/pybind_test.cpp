#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <iostream>
#include "planning_core/inc/ilqr_planner.h"
namespace py = pybind11; // 定义命名空间别名

class TestPybind{
public:
    TestPybind()=delete;
    TestPybind(double k){
        k_=k;
        std::cout<<"k= "<<k<<std::endl;
    }
    double k_;
};


static py::bytes RunIlqrOptimizer(const std::string &ilqr_proto_msg,
                                     const std::string &ilqr_params) {
  IlqrProtoMsg proto_msg{};
  IlqrParams params{};
  proto_msg.ParseFromString(ilqr_proto_msg);
  params.ParseFromString(ilqr_params);
    IlqrOptimizer ilqr_optimizer(params);
  ilqr_optimizer.Process(proto_msg);
  return py::bytes(proto_msg.SerializeAsString());
}

PYBIND11_MODULE(pybind_test, mod) {
    py::class_<TestPybind>(mod,"TestPybind")
        .def(py::init<double>()) 
        .def_readwrite("k", &TestPybind::k_);
    mod.def("RunIlqrOptimizer", &RunIlqrOptimizer, "RunIlqrOptimizer");
}

