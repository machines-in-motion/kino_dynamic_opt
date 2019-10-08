/**
 * @file PyMomentumopt.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_params(py::module& module);
void init_setting(py::module& module);
void init_terrain(py::module& module);
void init_contacts(py::module& module);
void init_dynamics(py::module& module);
void init_kinematics(py::module& module);


PYBIND11_MODULE(pymomentum, m) {
  init_params(m);
  init_setting(m);
  init_terrain(m);
  init_contacts(m);
  init_dynamics(m);
  init_kinematics(m);
}
