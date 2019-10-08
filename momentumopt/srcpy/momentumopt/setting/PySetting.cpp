/**
 * @file PySetting.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/setting/PlannerSetting.hpp>

namespace py = pybind11;
using namespace momentumopt;

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
	.def("get", (const Eigen::Ref<const Eigen::VectorXd> (PlannerSetting::*)(PlannerVectorParam) const) &PlannerSetting::get);
}
