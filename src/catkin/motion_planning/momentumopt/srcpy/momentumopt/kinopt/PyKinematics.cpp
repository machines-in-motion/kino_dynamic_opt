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

#include <momentumopt/kinopt/KinematicsState.hpp>

namespace py = pybind11;
using namespace momentumopt;

PYBIND11_MAKE_OPAQUE(std::vector<KinematicsState>);

void init_kinematics(py::module &m)
{
  // binding of stl containers
  py::bind_vector<std::vector<KinematicsState>>(m, "KinStateVector");

  // binding of robot posture
  py::class_<RobotPosture>(m, "RobotPosture")
    .def(py::init<int&>())
    .def_property("base_position", (const Eigen::Vector3d& (RobotPosture::*)(void) const) &RobotPosture::basePosition, (void (RobotPosture::*)(const Eigen::Vector3d&)) &RobotPosture::basePosition)
    .def_property("joint_positions", (const Eigen::VectorXd& (RobotPosture::*)(void) const) &RobotPosture::jointPositions, (void (RobotPosture::*)(const Eigen::VectorXd&)) &RobotPosture::jointPositions)
    .def_property("base_orientation", (const Eigen::Quaternion<double>& (RobotPosture::*)(void) const) &RobotPosture::baseOrientation, (void (RobotPosture::*)(const Eigen::Quaternion<double>&)) &RobotPosture::baseOrientation)
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
    .def_property("com", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::centerOfMass, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::centerOfMass)
    .def_property("lmom", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::linearMomentum, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::linearMomentum)
    .def_property("amom", (const Eigen::Vector3d& (KinematicsState::*)(void) const) &KinematicsState::angularMomentum, (void (KinematicsState::*)(const Eigen::Vector3d&)) &KinematicsState::angularMomentum)
	.def_property("robot_posture", (const RobotPosture& (KinematicsState::*)(void) const) &KinematicsState::robotPosture, (void (KinematicsState::*)(const RobotPosture&)) &KinematicsState::robotPosture)
	.def_property("robot_velocity", (const RobotVelocity& (KinematicsState::*)(void) const) &KinematicsState::robotVelocity, (void (KinematicsState::*)(const RobotVelocity&)) &KinematicsState::robotVelocity)
	.def_property("robot_acceleration", (const RobotAcceleration& (KinematicsState::*)(void) const) &KinematicsState::robotAcceleration, (void (KinematicsState::*)(const RobotAcceleration&)) &KinematicsState::robotAcceleration)
    .def("__repr__", [](const KinematicsState &kin_state) { return kin_state.toString(); } );

  // binding of kinematics sequence
  py::class_<KinematicsSequence>(m, "KinematicsSequence")
    .def(py::init<>())
    .def("size", &KinematicsSequence::size)
    .def("clean", &KinematicsSequence::clean)
    .def("resize", &KinematicsSequence::resize)
    .def_property("kinematicsStates", (const std::vector<KinematicsState>& (KinematicsSequence::*)(void) const) &KinematicsSequence::kinematicsSequence, (void (KinematicsSequence::*)(const std::vector<KinematicsState>&)) &KinematicsSequence::kinematicsSequence)
    .def("__repr__", [](const KinematicsSequence &kin_seq) { return kin_seq.toString(); } );

}
