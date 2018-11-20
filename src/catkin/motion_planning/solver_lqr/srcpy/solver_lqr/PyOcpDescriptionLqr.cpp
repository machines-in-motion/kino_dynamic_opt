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

#include <Eigen/Dense>
#include <solver_lqr/PyOcpBase.hpp>
#include <solver_lqr/OcpDescription.hpp>

namespace py = pybind11;
using namespace solverlqr;

void init_ocp_description_lqr(py::module &m)
{
  // binding of state base class
  py::class_<StateBase>(m, "StateBase")
    .def(py::init<>());

  // binding of control base class
  py::class_<ControlBase>(m, "ControlBase")
    .def(py::init<>());

  // binding of OCP description
  py::class_<OcpBase, PyOcpBase>(m, "OcpBase")
    .def(py::init<>())
    .def("dynamics", &OcpBase::dynamics)
    .def("objective", &OcpBase::objective)
    .def("configure", &OcpBase::configure)
    .def("initialize", &OcpBase::initialize);
}
