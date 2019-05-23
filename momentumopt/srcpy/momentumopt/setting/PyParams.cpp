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

#include <momentumopt/setting/PlannerParams.hpp>

namespace py = pybind11;
using namespace momentumopt;

void init_params(py::module &m)
{
  // binding integer parameters
  py::enum_<PlannerIntParam>(m, "PlannerIntParam")
  .value("PlannerIntParam_NumDofs", PlannerIntParam_NumDofs)
  .value("PlannerIntParam_NumTimesteps", PlannerIntParam_NumTimesteps)
  .value("PlannerIntParam_NumViapoints", PlannerIntParam_NumViapoints)
	.value("PlannerIntParam_NumActiveDofs", PlannerIntParam_NumActiveDofs)
	.value("PlannerIntParam_NumSubsamples", PlannerIntParam_NumSubsamples)
	.value("PlannerIntParam_KinDynIterations", PlannerIntParam_KinDynIterations)
	.value("PlannerIntParam_MaxNumTimeIterations", PlannerIntParam_MaxNumTimeIterations)
	.value("PlannerIntParam_NumExtendedActiveDofs", PlannerIntParam_NumExtendedActiveDofs)
	.value("PlannerIntParam_NumActiveEndeffectors", PlannerIntParam_NumActiveEndeffectors)
	.value("PlannerIntParam_MaxKinConvergenceIterations", PlannerIntParam_MaxKinConvergenceIterations)
  .export_values();

  // binding boolean parameters
  py::enum_<PlannerBoolParam>(m, "PlannerBoolParam")
  .value("PlannerBoolParam_StoreData", PlannerBoolParam_StoreData)
	.value("PlannerBoolParam_DisplayMotion", PlannerBoolParam_DisplayMotion)
	.value("PlannerBoolParam_LoadKinematics", PlannerBoolParam_LoadKinematics)
  .value("PlannerBoolParam_IsTimeHorizonFixed", PlannerBoolParam_IsTimeHorizonFixed)
  .value("PlannerBoolParam_IsFrictionConeLinear", PlannerBoolParam_IsFrictionConeLinear)
	.value("PlannerBoolParam_UseDefaultSolverSetting", PlannerBoolParam_UseDefaultSolverSetting)
  .export_values();

  // binding double parameters
  py::enum_<PlannerDoubleParam>(m, "PlannerDoubleParam")
  .value("PlannerDoubleParam_Gravity", PlannerDoubleParam_Gravity)
  .value("PlannerDoubleParam_TimeStep", PlannerDoubleParam_TimeStep)
	.value("PlannerDoubleParam_RobotMass", PlannerDoubleParam_RobotMass)
	.value("PlannerDoubleParam_FloorHeight", PlannerDoubleParam_FloorHeight)
	.value("PlannerDoubleParam_RobotWeight", PlannerDoubleParam_RobotWeight)
  .value("PlannerDoubleParam_TimeHorizon", PlannerDoubleParam_TimeHorizon)
	.value("PlannerDoubleParam_MinRelHeight", PlannerDoubleParam_MinRelHeight)
	.value("PlannerDoubleParam_WeightArmTorque", PlannerDoubleParam_WeightArmTorque)
	.value("PlannerDoubleParam_WeightLegTorque", PlannerDoubleParam_WeightLegTorque)
	.value("PlannerDoubleParam_MassTimesGravity", PlannerDoubleParam_MassTimesGravity)
	.value("PlannerDoubleParam_WeightTimePenalty", PlannerDoubleParam_WeightTimePenalty)
	.value("PlannerDoubleParam_FrictionCoefficient", PlannerDoubleParam_FrictionCoefficient)
	.value("PlannerDoubleParam_KinConvergenceTolerance", PlannerDoubleParam_KinConvergenceTolerance)
	.value("PlannerDoubleParam_MaxTimeResidualTolerance", PlannerDoubleParam_MaxTimeResidualTolerance)
	.value("PlannerDoubleParam_WeightTimeRegularization", PlannerDoubleParam_WeightTimeRegularization)
  .value("PlannerDoubleParam_MinTimeResidualImprovement", PlannerDoubleParam_MinTimeResidualImprovement)
  .value("PlannerDoubleParam_SwingTrajViaZ", PlannerDoubleParam_SwingTrajViaZ)
  .value("PlannerDoubleParam_WeightLinMomentumTracking", PlannerDoubleParam_WeightLinMomentumTracking)
  .value("PlannerDoubleParam_WeightAngMomentumTracking", PlannerDoubleParam_WeightAngMomentumTracking)
  .value("PlannerDoubleParam_WeightEndEffContact", PlannerDoubleParam_WeightEndEffContact)
  .value("PlannerDoubleParam_WeightEndEffTracking", PlannerDoubleParam_WeightEndEffTracking)
  .value("PlannerDoubleParam_PGainEndEffTracking", PlannerDoubleParam_PGainEndEffTracking)
  .value("PlannerDoubleParam_PGainComTracking", PlannerDoubleParam_PGainComTracking)
  .value("PlannerDoubleParam_WeightJointReg", PlannerDoubleParam_WeightJointReg)
  .value("PlannerDoubleParam_PGainOrientationTracking", PlannerDoubleParam_PGainOrientationTracking)
  .export_values();

  // binding string parameters
  py::enum_<PlannerStringParam>(m, "PlannerStringParam")
  .value("PlannerStringParam_ConfigFile", PlannerStringParam_ConfigFile)
  .value("PlannerStringParam_SaveDynamicsFile", PlannerStringParam_SaveDynamicsFile)
	.value("PlannerStringParam_SaveKinematicsFile", PlannerStringParam_SaveKinematicsFile)
	.value("PlannerStringParam_DefaultSolverSettingFile", PlannerStringParam_DefaultSolverSettingFile)
  .export_values();

  // binding Eigen::VectorXi vector parameters
  py::enum_<PlannerIntVectorParam>(m, "PlannerIntVectorParam")
  .value("PlannerIntVectorParam_ActiveDofs", PlannerIntVectorParam_ActiveDofs)
	.value("PlannerIntVectorParam_ExtendedActiveDofs", PlannerIntVectorParam_ExtendedActiveDofs)
	.export_values();

  // binding Eigen::VectorXd vector parameters
  py::enum_<PlannerVectorParam>(m, "PlannerVectorParam")
  .value("PlannerVectorParam_TimeRange", PlannerVectorParam_TimeRange)
  .value("PlannerVectorParam_TorqueRange", PlannerVectorParam_TorqueRange)
	.value("PlannerVectorParam_ExternalForce", PlannerVectorParam_ExternalForce)
	.value("PlannerVectorParam_GravityVector", PlannerVectorParam_GravityVector)
	.value("PlannerVectorParam_CenterOfMassMotion", PlannerVectorParam_CenterOfMassMotion)
	.value("PlannerVectorParam_MaxEndeffectorLengths", PlannerVectorParam_MaxEndeffectorLengths)

  .value("PlannerVectorParam_WeightArmForce", PlannerVectorParam_WeightArmForce)
  .value("PlannerVectorParam_WeightLegForce", PlannerVectorParam_WeightLegForce)
	.value("PlannerVectorParam_WeightArmForceRate", PlannerVectorParam_WeightArmForceRate)
	.value("PlannerVectorParam_WeightLegForceRate", PlannerVectorParam_WeightLegForceRate)
	.value("PlannerVectorParam_WeightCenterOfMass", PlannerVectorParam_WeightCenterOfMass)
	.value("PlannerVectorParam_WeightLinearMomentum", PlannerVectorParam_WeightLinearMomentum)
  .value("PlannerVectorParam_WeightAngularMomentum", PlannerVectorParam_WeightAngularMomentum)
  .value("PlannerVectorParam_WeightLinearMomentumRate", PlannerVectorParam_WeightLinearMomentumRate)
	.value("PlannerVectorParam_WeightAngularMomentumRate", PlannerVectorParam_WeightAngularMomentumRate)
	.value("PlannerVectorParam_WeightFinalLinearMomentum", PlannerVectorParam_WeightFinalLinearMomentum)
	.value("PlannerVectorParam_WeightCenterOfMassViapoint", PlannerVectorParam_WeightCenterOfMassViapoint)
	.value("PlannerVectorParam_WeightFinalAngularMomentum", PlannerVectorParam_WeightFinalAngularMomentum)
	.value("PlannerVectorParam_WeightDynamicTrackingLinearMomentum", PlannerVectorParam_WeightDynamicTrackingLinearMomentum)
	.value("PlannerVectorParam_WeightDynamicTrackingAngularMomentum", PlannerVectorParam_WeightDynamicTrackingAngularMomentum)

  .value("PlannerVectorParam_MinJointAngles", PlannerVectorParam_MinJointAngles)
	.value("PlannerVectorParam_MaxJointAngles", PlannerVectorParam_MaxJointAngles)
	.value("PlannerVectorParam_KinematicDefaultJointPositions", PlannerVectorParam_KinematicDefaultJointPositions)

	.value("PlannerVectorParam_WeightJointVelocity", PlannerVectorParam_WeightJointVelocity)
	.value("PlannerVectorParam_WeightJointAcceleration", PlannerVectorParam_WeightJointAcceleration)
	.value("PlannerVectorParam_WeightKinematicTrackingCenterOfMass", PlannerVectorParam_WeightKinematicTrackingCenterOfMass)
	.value("PlannerVectorParam_WeightKinematicDefaultJointPositions", PlannerVectorParam_WeightKinematicDefaultJointPositions)
	.value("PlannerVectorParam_WeightKinematicTrackingLinearMomentum", PlannerVectorParam_WeightKinematicTrackingLinearMomentum)
	.value("PlannerVectorParam_WeightKinematicTrackingAngularMomentum", PlannerVectorParam_WeightKinematicTrackingAngularMomentum)
	.value("PlannerVectorParam_WeightKinematicTrackingBaseOrientation", PlannerVectorParam_WeightKinematicTrackingBaseOrientation)
	.value("PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate", PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate)
	.value("PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate", PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate)
	.value("PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition", PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition)

  .export_values();
}
