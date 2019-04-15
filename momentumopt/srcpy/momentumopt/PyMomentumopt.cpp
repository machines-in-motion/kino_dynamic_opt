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
