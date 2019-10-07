/**
 * @file PySolverLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_params_lqr(py::module& module);
void init_setting_lqr(py::module& module);
void init_algorithm_lqr(py::module& module);
void init_ocp_description_lqr(py::module& module);

PYBIND11_MODULE(pysolverlqr, m) {
  init_params_lqr(m);
  init_setting_lqr(m);
  init_algorithm_lqr(m);
  init_ocp_description_lqr(m);
}
