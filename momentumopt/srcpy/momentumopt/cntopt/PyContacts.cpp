/**
 * @file PyContacts.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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

  //binding of contact type enum
  py::enum_<ContactType>(m, "ContactType")
    .value("FreeContact", ContactType::FreeContact)
    .value("FlatContact", ContactType::FlatContact)
    .value("FullContact", ContactType::FullContact)
    .export_values();

 
  // binding of contacts state
  py::class_<ContactState>(m, "ContactState")
    .def(py::init<>())
    .def_property("start_time", (const double& (ContactState::*)(void) const) &ContactState::contactActivationTime, (void (ContactState::*)(const double&)) &ContactState::contactActivationTime)
    .def_property("end_time", (const double& (ContactState::*)(void) const) &ContactState::contactDeactivationTime, (void (ContactState::*)(const double&)) &ContactState::contactDeactivationTime)
    .def_property("position", (const Eigen::Vector3d& (ContactState::*)(void) const) &ContactState::contactPosition, (void (ContactState::*)(const Eigen::Vector3d&)) &ContactState::contactPosition)
    .def_property("contactType", (const ContactType& (ContactState::*)(void) const) &ContactState::contactType, (void (ContactState::*)(const ContactType&)) &ContactState::contactType)
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
