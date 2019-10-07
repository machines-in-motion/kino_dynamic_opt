/**
 * @file PySettingLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <solver_lqr/SolverLqrSetting.hpp>

namespace py = pybind11;
using namespace solverlqr;

void init_setting_lqr(py::module &m)
{

  // binding of solver lqr setting
  py::class_<SolverLqrSetting>(m, "SolverLqrSetting")
    .def(py::init<>())
    .def("initialize", &SolverLqrSetting::initialize, py::arg("cfg_file"), py::arg("stgs_vars_yaml") = "solverlqr_variables")
    .def("get", (const int& (SolverLqrSetting::*)(SolverLqrIntParam) const) &SolverLqrSetting::get)
    .def("get", (const bool& (SolverLqrSetting::*)(SolverLqrBoolParam) const) &SolverLqrSetting::get)
    .def("get", (const double& (SolverLqrSetting::*)(SolverLqrDoubleParam) const) &SolverLqrSetting::get)
	.def("get", (const std::string& (SolverLqrSetting::*)(SolverLqrStringParam) const) &SolverLqrSetting::get)
	.def("get", (const Eigen::VectorXd& (SolverLqrSetting::*)(SolverLqrVectorParam) const) &SolverLqrSetting::get);
}
