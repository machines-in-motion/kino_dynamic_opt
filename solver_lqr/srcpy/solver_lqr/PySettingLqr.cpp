/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
