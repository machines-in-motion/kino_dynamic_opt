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

#include <momentumopt/cntopt/ContactPlanFromFile.hpp>

namespace py = pybind11;
using namespace momentumopt;

PYBIND11_MAKE_OPAQUE(std::vector<ContactState>)

void init_contacts(py::module &m)
{
  // binding of stl containers
  py::bind_vector<std::vector<ContactState>>(m, "CntStateVector");

  // binding of contacts state
  py::class_<ContactState>(m, "ContactState")
    .def(py::init<>())
    .def_property("start_time", (const double& (ContactState::*)(void) const) &ContactState::contactActivationTime, (void (ContactState::*)(const double&)) &ContactState::contactActivationTime)
    .def_property("end_time", (const double& (ContactState::*)(void) const) &ContactState::contactDeactivationTime, (void (ContactState::*)(const double&)) &ContactState::contactDeactivationTime)
    .def_property("position", (const Eigen::Vector3d& (ContactState::*)(void) const) &ContactState::contactPosition, (void (ContactState::*)(const Eigen::Vector3d&)) &ContactState::contactPosition)
    .def("__repr__", [](const ContactState &cnt_state) { return cnt_state.toString(); } );

  // binding of contacts sequence
  py::class_<ContactSequence>(m, "ContactSequence")
    .def(py::init<>())
    .def("contact_states", (const std::vector<ContactState>& (ContactSequence::*)(int) const) &ContactSequence::endeffectorContacts)
    .def("eff_num", (const long (ContactSequence::*)() const) &ContactSequence::endeffectorNum)
    .def("__repr__", [](const ContactSequence &cnt_seq) { return cnt_seq.toString(); } );

  py::class_<ContactPlanInterface, ContactPlanFromFile>(m, "ContactPlanFromFile")
    .def(py::init<>())
    .def("initialize", &ContactPlanInterface::initialize)
    .def("optimize", &ContactPlanInterface::optimize)
    .def("contactSequence", (const ContactSequence& (ContactPlanInterface::*)(void) const) &ContactPlanInterface::contactSequence);
}
