/**
 * @file PySetting.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/setting/PlannerSetting.hpp>

namespace py = pybind11;
using namespace momentumopt;

static void setIntParam(PlannerSetting& self,const PlannerIntParam& param, const int value){
  self.get(param) = value;
}
static void setBoolParam(PlannerSetting& self,const PlannerBoolParam& param, const bool value){
  self.get(param) = value;
}
static void setDoubleParam(PlannerSetting& self,const PlannerDoubleParam& param, const double value){
  self.get(param) = value;
}
static void setStringParam(PlannerSetting& self,const PlannerStringParam& param, const std::string& value){
  self.get(param) = value;
}
static void setCVectorParam(PlannerSetting& self,const PlannerCVectorParam& param, const std::vector<Eigen::VectorXd>& value){
  self.get(param) = value;
}
static void setVectorParam(PlannerSetting& self,const PlannerVectorParam& param, const Eigen::Ref<const Eigen::VectorXd> value){
  self.get(param) = value;
}

void init_setting(py::module &m)
{
  py::enum_<Heuristic>(m, "Heuristic")
    .value("TrustRegion", Heuristic::TrustRegion)
    .value("SoftConstraint", Heuristic::SoftConstraint)
	.value("TimeOptimization", Heuristic::TimeOptimization)
    .export_values();

  // binding of dynamics state
  py::class_<PlannerSetting>(m, "PlannerSetting")
  .def(py::init<>())
  .def("initialize", &PlannerSetting::initialize, py::arg("cfg_file"), py::arg("planner_vars_yaml") = "planner_variables")
  .def("get", (const int& (PlannerSetting::*)(PlannerIntParam) const) &PlannerSetting::get)
  .def("get", (const bool& (PlannerSetting::*)(PlannerBoolParam) const) &PlannerSetting::get)
  .def("get", (const double& (PlannerSetting::*)(PlannerDoubleParam) const) &PlannerSetting::get)
	.def("get", (const std::string& (PlannerSetting::*)(PlannerStringParam) const) &PlannerSetting::get)
  .def("get", (const std::vector<Eigen::VectorXd>& (PlannerSetting::*)(PlannerCVectorParam) const) &PlannerSetting::get)
	.def("get", (const Eigen::Ref<const Eigen::VectorXd> (PlannerSetting::*)(PlannerVectorParam) const) &PlannerSetting::get)
  .def("set", &setIntParam, py::arg("IntParameter"), py::arg("value"))
  .def("set", &setBoolParam, py::arg("BoolParameter"), py::arg("value"))
  .def("set", &setDoubleParam, py::arg("DoubleParameter"), py::arg("value"))
  .def("set", &setStringParam, py::arg("StringParameter"), py::arg("value"))
  .def("set", &setCVectorParam, py::arg("CVectorParameter"), py::arg("value"))
  .def("set", &setVectorParam, py::arg("VectorParameter"), py::arg("value"));
}


