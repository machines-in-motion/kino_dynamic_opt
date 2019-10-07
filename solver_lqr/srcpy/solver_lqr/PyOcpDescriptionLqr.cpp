/**
 * @file PyOcpDescriptionLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
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
