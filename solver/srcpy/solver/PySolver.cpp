/**
 * @file PySolver.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_setting(py::module& module);


PYBIND11_MODULE(pysolver, m) {
  init_setting(m);
}
