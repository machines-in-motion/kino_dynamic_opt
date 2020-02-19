/**
 * @file PyDynamics.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/dynopt/DynamicsFeedback.hpp>
#include <momentumopt/dynopt/DynamicsOptimizer.hpp>

namespace py = pybind11;
using namespace momentumopt;

PYBIND11_MAKE_OPAQUE(std::vector<DynamicsState>)

static void setEffPosition(DynamicsState& self, const int effId, const Eigen::Vector3d& pos){
  self.endeffectorPosition(effId) = pos;
}
static void setEffForce(DynamicsState& self, const int effId, const Eigen::Vector3d& force){
  self.endeffectorForce(effId) = force;
}
static void setEffTorque(DynamicsState& self, const int effId, const Eigen::Vector3d& torque){
  self.endeffectorPosition(effId) = torque;
}

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
    .def_property("amomd", (const Eigen::Vector3d& (DynamicsState::*)(void) const) &DynamicsState::angularMomentumRate, (void (DynamicsState::*)(const Eigen::Vector3d&)) &DynamicsState::angularMomentumRate)
    .def("fillInitialRobotState", &DynamicsState::fillInitialRobotState, py::arg("cfg_file"), py::arg("robot_state") = "initial_robot_configuration")

    .def("eff", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorPosition)
    .def("effCoP", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorCoP)
    .def("effForce", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorForce)
    .def("effTorque", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorTorque)

    .def("effNum", (const long (DynamicsState::*)() const) &DynamicsState::endeffectorNum)

    .def("effPosition", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorPosition)
    .def("effVelocity", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorVelocity)
    .def("effAcceleration", (const Eigen::Vector3d& (DynamicsState::*)(int) const) &DynamicsState::endeffectorAcceleration)
    .def("effOrientation", (const Eigen::Quaternion<double>& (DynamicsState::*)(int) const) &DynamicsState::endeffectorOrientation)

    .def("setEffPosition", &setEffPosition, py::arg("EffId"), py::arg("position"))
    .def("setEffForce", &setEffForce, py::arg("EffId"), py::arg("force"))
    .def("setEffTorque", &setEffTorque, py::arg("EffId"), py::arg("torque"))

    .def("__repr__", [](const DynamicsState &dyn_state) { return dyn_state.toString(); } );

  // binding of dynamics sequence
  py::class_<DynamicsSequence>(m, "DynamicsSequence")
    .def(py::init<>())
    .def("size", &DynamicsSequence::size)
    .def("clean", &DynamicsSequence::clean)
    .def("resize", &DynamicsSequence::resize)
    .def_property("dynamics_states", (const std::vector<DynamicsState>& (DynamicsSequence::*)(void) const) &DynamicsSequence::dynamicsSequence, (void (DynamicsSequence::*)(const std::vector<DynamicsState>&)) &DynamicsSequence::dynamicsSequence)
    .def("__repr__", [](const DynamicsSequence &dyn_seq) { return dyn_seq.toString(); } );

  // binding of dynamics optimizer
  py::class_<DynamicsOptimizer>(m, "DynamicsOptimizer")
    .def(py::init<>())
    .def("initialize", &DynamicsOptimizer::initialize)
    .def("optimize", &DynamicsOptimizer::optimize, py::arg("ini_state"), py::arg("contact_plan"), py::arg("kin_sequence"), py::arg("update_tracking_objective") = false)
    .def("dynamicsSequence", (const DynamicsSequence& (DynamicsOptimizer::*)(void) const) &DynamicsOptimizer::dynamicsSequence)
    .def("solveTime", &DynamicsOptimizer::solveTime);

  // binding of dynamics feedback wrapper
  py::class_<DynamicsFeedbackWrapper>(m, "DynamicsFeedback")
    .def(py::init<>())
    .def("initialize", &DynamicsFeedbackWrapper::initialize)
    .def("optimize", &DynamicsFeedbackWrapper::optimize)
	.def("forceGain", &DynamicsFeedbackWrapper::forceGain);

}
