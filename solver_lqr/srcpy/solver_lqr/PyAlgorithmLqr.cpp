/**
 * @file PyAlgorithmLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <solver_lqr/SolverLqr.hpp>

namespace py = pybind11;
using namespace solverlqr;

void init_algorithm_lqr(py::module &m)
{

  // binding of solver lqr algorithm
  py::class_<SolverLqr>(m, "SolverLqr")
    .def(py::init<>())
    .def("initialize", &SolverLqr::initialize)
    .def("optimize", &SolverLqr::optimize);
}
