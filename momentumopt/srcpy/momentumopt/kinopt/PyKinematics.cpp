/**
 * @file PyKinematics.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <momentumopt/kinopt/KinematicsState.hpp>
#include <momentumopt/kinopt/KinematicsOptimizer.hpp>
#include <momentumopt/kinopt/PyKinematicsInterface.hpp>

namespace py = pybind11;
using namespace momentumopt;

PYBIND11_MAKE_OPAQUE(std::vector<KinematicsState>)
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::MatrixXd>)
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>)

void init_kinematics(py::module &m)
{
  // binding of stl containers
  py::bind_vector<std::vector<Eigen::Vector3d>>(m, "ArrayVector3d");
  py::bind_vector<std::vector<KinematicsState>>(m, "KinStateVector");
  py::bind_vector<std::vector<Eigen::MatrixXd>>(m, "JacobianVector");

  // binding of robot posture
  py::class_<RobotPosture>(m, "RobotPosture")
    .def(py::init<int&>())
    .def_property_readonly("generalized_joint_positions", &RobotPosture::generalizedJointPositions)
    .def_property("base_position", (const Eigen::Vector3d& (RobotPosture::*)(void) const) &RobotPosture::basePosition, (void (RobotPosture::*)(const Eigen::Vector3d&)) &RobotPosture::basePosition)
    .def_property("base_orientation", (const Eigen::Vector4d (RobotPosture::*)(void) const) &RobotPosture::pyBaseOrientation, (void (RobotPosture::*)(const Eigen::Vector4d)) &RobotPosture::pyBaseOrientation)
    .def_property("joint_positions", (const Eigen::VectorXd& (RobotPosture::*)(void) const) &RobotPosture::jointPositions, (void (RobotPosture::*)(const Eigen::VectorXd&)) &RobotPosture::jointPositions)
    .def("__repr__", [](const RobotPosture &robot_posture) { return robot_posture.toString(); } );

  // binding of robot velocity
  py::class_<RobotVelocity>(m, "RobotVelocity")
    .def(py::init<int&>())
	  .def_property("generalized_joint_velocities", (const Eigen::VectorXd& (RobotVelocity::*)(void) const) &RobotVelocity::generalizedJointVelocities, (void (RobotVelocity::*)(const Eigen::VectorXd&)) &RobotVelocity::generalizedJointVelocities)
    .def_property("joint_velocities", (const Eigen::Ref<const Eigen::VectorXd> (RobotVelocity::*)(void) const) &RobotVelocity::jointVelocities, (void (RobotVelocity::*)(const Eigen::Ref<const Eigen::VectorXd>)) &RobotVelocity::jointVelocities)
    .def_property("base_linear_velocity", (const Eigen::Ref<const Eigen::Vector3d> (RobotVelocity::*)(void) const) &RobotVelocity::baseLinearVelocity, (void (RobotVelocity::*)(const Eigen::Ref<const Eigen::Vector3d>)) &RobotVelocity::baseLinearVelocity)
    .def_property("base_angular_velocity", (const Eigen::Ref<const Eigen::Vector3d> (RobotVelocity::*)(void) const) &RobotVelocity::baseAngularVelocity, (void (RobotVelocity::*)(const Eigen::Ref<const Eigen::Vector3d>)) &RobotVelocity::baseAngularVelocity)
    .def("__repr__", [](const RobotVelocity &robot_velocity) { return robot_velocity.toString(); } );

  // binding of robot acceleration
  py::class_<RobotAcceleration>(m, "RobotAcceleration")
    .def(py::init<int&>())
	.def_property("generalized_joint_accelerations", (const Eigen::VectorXd& (RobotAcceleration::*)(void) const) &RobotAcceleration::generalizedJointAccelerations, (void (RobotAcceleration::*)(const Eigen::VectorXd&)) &RobotAcceleration::generalizedJointAccelerations)
    .def_property("joint_accelerations", (const Eigen::Ref<const Eigen::VectorXd> (RobotAcceleration::*)(void) const) &RobotAcceleration::jointAccelerations, (void (RobotAcceleration::*)(const Eigen::Ref<const Eigen::VectorXd>)) &RobotAcceleration::jointAccelerations)
    .def_property("base_linear_acceleration", (const Eigen::Ref<const Eigen::Vector3d> (RobotAcceleration::*)(void) const) &RobotAcceleration::baseLinearAcceleration, (void (RobotAcceleration::*)(const Eigen::Ref<const Eigen::Vector3d>)) &RobotAcceleration::baseLinearAcceleration)
    .def_property("base_angular_acceleration", (const Eigen::Ref<const Eigen::Vector3d> (RobotAcceleration::*)(void) const) &RobotAcceleration::baseAngularAcceleration, (void (RobotAcceleration::*)(const Eigen::Ref<const Eigen::Vector3d>)) &RobotAcceleration::baseAngularAcceleration)
    .def("__repr__", [](const RobotAcceleration &robot_acceleration) { return robot_acceleration.toString(); } );

  // binding of kinematics state
  py::class_<KinematicsState>(m, "KinematicsState")
    .def(py::init<int&>())
    .def_property_readonly("numJoints", &KinematicsState::numJoints)
    .def_property("com", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::centerOfMass, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::centerOfMass)
    .def_property("lmom", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::linearMomentum, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::linearMomentum)
    .def_property("amom", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::angularMomentum, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::angularMomentum)
	.def_property("robot_posture", (const RobotPosture& (KinematicsState::*)(void) const) &KinematicsState::robotPosture, (void (KinematicsState::*)(const RobotPosture&)) &KinematicsState::robotPosture)
	.def_property("robot_velocity", (const RobotVelocity& (KinematicsState::*)(void) const) &KinematicsState::robotVelocity, (void (KinematicsState::*)(const RobotVelocity&)) &KinematicsState::robotVelocity)
	.def_property("robot_acceleration", (const RobotAcceleration& (KinematicsState::*)(void) const) &KinematicsState::robotAcceleration, (void (KinematicsState::*)(const RobotAcceleration&)) &KinematicsState::robotAcceleration)
    .def_property("endeffector_positions", (const std::vector<Eigen::Vector3d>& (KinematicsState::*)(void) const) &KinematicsState::pyEndeffectorPositions, (void (KinematicsState::*)(const std::vector<Eigen::Vector3d>&)) &KinematicsState::pyEndeffectorPositions)
    .def("__repr__", [](const KinematicsState &kin_state) { return kin_state.toString(); } );

  // binding of kinematics sequence
  py::class_<KinematicsSequence>(m, "KinematicsSequence")
    .def(py::init<>())
    .def("size", &KinematicsSequence::size)
    .def("clean", &KinematicsSequence::clean)
    .def("resize", &KinematicsSequence::resize)
    .def_property("kinematics_states", (const std::vector<KinematicsState>& (KinematicsSequence::*)(void) const) &KinematicsSequence::kinematicsSequence, (void (KinematicsSequence::*)(const std::vector<KinematicsState>&)) &KinematicsSequence::kinematicsSequence)
    .def("__repr__", [](const KinematicsSequence &kin_seq) { return kin_seq.toString(); } );

  // binding of kinematics interface
  py::class_<KinematicsInterface, PyKinematicsInterface>(m, "KinematicsInterface")
    .def(py::init<>())
	.def("initialize", &KinematicsInterface::initialize)
	.def("displayPosture", &KinematicsInterface::displayPosture)
	.def("logarithmicMap", &KinematicsInterface::logarithmicMap)
	.def("integratePosture", &KinematicsInterface::integratePosture)
	.def("differentiatePostures", &KinematicsInterface::differentiatePostures)
    .def("updateJacobiansAndState", &KinematicsInterface::updateJacobiansAndState)
	.def_property("constraints_vector", (const Eigen::VectorXd& (KinematicsInterface::*)(void) const) &KinematicsInterface::constraintsVector, (void (KinematicsInterface::*)(const Eigen::VectorXd&)) &KinematicsInterface::constraintsVector)
	.def_property("constraints_matrix", (const Eigen::MatrixXd& (KinematicsInterface::*)(void) const) &KinematicsInterface::constraintsMatrix, (void (KinematicsInterface::*)(const Eigen::MatrixXd&)) &KinematicsInterface::constraintsMatrix)
	.def_property("center_of_mass_jacobian", (const Eigen::MatrixXd& (KinematicsInterface::*)(void) const) &KinematicsInterface::centerOfMassJacobian, (void (KinematicsInterface::*)(const Eigen::MatrixXd&)) &KinematicsInterface::centerOfMassJacobian)
    .def_property("centroidal_momentum_matrix", (const Eigen::MatrixXd& (KinematicsInterface::*)(void) const) &KinematicsInterface::centroidalMomentumMatrix, (void (KinematicsInterface::*)(const Eigen::MatrixXd&)) &KinematicsInterface::centroidalMomentumMatrix)
    .def_property("endeffector_jacobians", (const std::vector<Eigen::MatrixXd>& (KinematicsInterface::*)(void) const) &KinematicsInterface::endeffectorJacobians, (void (KinematicsInterface::*)(const std::vector<Eigen::MatrixXd>&)) &KinematicsInterface::endeffectorJacobians)
    .def_property("centroidal_momentum_matrix_variation", (const Eigen::MatrixXd& (KinematicsInterface::*)(void) const) &KinematicsInterface::centroidalMomentumMatrixVariation, (void (KinematicsInterface::*)(const Eigen::MatrixXd&)) &KinematicsInterface::centroidalMomentumMatrixVariation);

  // binding of kinematics optimizer
  py::class_<KinematicsOptimizer>(m, "KinematicsOptimizer")
    .def(py::init<>())
    .def("initialize", &KinematicsOptimizer::initialize)
	.def("optimize", &KinematicsOptimizer::optimize, py::arg("ini_state"), py::arg("contact_plan"), py::arg("dyn_sequence"), py::arg("is_first_kinopt"))
    .def("kinematicsSequence", (const KinematicsSequence& (KinematicsOptimizer::*)(void) const) &KinematicsOptimizer::kinematicsSequence);
}
