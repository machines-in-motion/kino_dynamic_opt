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

#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/dynopt/DynamicsOptimizer.hpp>

namespace py = pybind11;
using namespace momentumopt;

PYBIND11_MAKE_OPAQUE(std::vector<DynamicsState>);

void init_dynamics(py::module &m)
{
  // binding of stl containers
  py::bind_vector<std::vector<DynamicsState>>(m, "DynStateVector");

  // binding of dynamics state
  py::class_<DynamicsState>(m, "DynamicsState")
    .def(py::init<>())
    .def_property("dt", (const double& (DynamicsState::*)(void) const) &DynamicsState::time, (void (DynamicsState::*)(const double&)) &DynamicsState::time)
    .def_property("com", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::centerOfMass, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::centerOfMass)
    .def_property("lmom", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::linearMomentum, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::linearMomentum)
    .def_property("amom", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::angularMomentum, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::angularMomentum)
    .def_property("lmomd", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::linearMomentumRate, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::linearMomentumRate)
    .def_property("amomd", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::angularMomentumRate, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::angularMomentumRate)
	.def("fillInitialRobotState", &DynamicsState::fillInitialRobotState, py::arg("cfg_file"), py::arg("robot_state") = "initial_robot_configuration")
    .def("__repr__", [](const DynamicsState &dyn_state) { return dyn_state.toString(); } );

  // binding of dynamics sequence
  py::class_<DynamicsSequence>(m, "DynamicsSequence")
    .def(py::init<>())
    .def("size", &DynamicsSequence::size)
    .def("clean", &DynamicsSequence::clean)
    .def("resize", &DynamicsSequence::resize)
    .def_property("dynamicsStates", (const std::vector<DynamicsState>& (DynamicsSequence::*)(void) const) &DynamicsSequence::dynamicsSequence, (void (DynamicsSequence::*)(const std::vector<DynamicsState>&)) &DynamicsSequence::dynamicsSequence)
    .def("__repr__", [](const DynamicsSequence &dyn_seq) { return dyn_seq.toString(); } );

  // binding of dynamics optimizer
  py::class_<DynamicsOptimizer>(m, "DynamicsOptimizer")
    .def(py::init<>())
    .def("initialize", &DynamicsOptimizer::initialize)
    .def("optimize", &DynamicsOptimizer::optimize, py::arg("ref_sequence"), py::arg("update_tracking_objective") = false)
    .def("dynamicsSequence", (const DynamicsSequence& (DynamicsOptimizer::*)(void) const) &DynamicsOptimizer::dynamicsSequence)
    .def("solveTime", &DynamicsOptimizer::solveTime);
}
