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
	.def("get", (const Eigen::Ref<const Eigen::VectorXd> (PlannerSetting::*)(PlannerVectorParam) const) &PlannerSetting::get);
}
