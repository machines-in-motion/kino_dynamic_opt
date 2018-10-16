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
    .value("PlannerIntParam_NumTimesteps", PlannerIntParam_NumTimesteps)
    .value("PlannerIntParam_NumViapoints", PlannerIntParam_NumViapoints)
	.value("PlannerIntParam_MaxNumTimeIterations", PlannerIntParam_MaxNumTimeIterations)
	.value("PlannerIntParam_NumActiveEndeffectors", PlannerIntParam_NumActiveEndeffectors)
    .export_values();

  // binding boolean parameters
  py::enum_<PlannerBoolParam>(m, "PlannerBoolParam")
    .value("PlannerBoolParam_IsTimeHorizonFixed", PlannerBoolParam_IsTimeHorizonFixed)
    .value("PlannerBoolParam_IsFrictionConeLinear", PlannerBoolParam_IsFrictionConeLinear)
	.value("PlannerBoolParam_StoreData", PlannerBoolParam_StoreData)
	.value("PlannerBoolParam_UseDefaultSolverSetting", PlannerBoolParam_UseDefaultSolverSetting)
    .export_values();

  // binding double parameters
  py::enum_<PlannerDoubleParam>(m, "PlannerDoubleParam")
    .value("PlannerDoubleParam_Gravity", PlannerDoubleParam_Gravity)
    .value("PlannerDoubleParam_TimeStep", PlannerDoubleParam_TimeStep)
	.value("PlannerDoubleParam_RobotMass", PlannerDoubleParam_RobotMass)
	.value("PlannerDoubleParam_RobotWeight", PlannerDoubleParam_RobotWeight)
    .value("PlannerDoubleParam_TimeHorizon", PlannerDoubleParam_TimeHorizon)
    .value("PlannerDoubleParam_MassTimesGravity", PlannerDoubleParam_MassTimesGravity)
	.value("PlannerDoubleParam_FrictionCoefficient", PlannerDoubleParam_FrictionCoefficient)
	.value("PlannerDoubleParam_MaxTimeResidualTolerance", PlannerDoubleParam_MaxTimeResidualTolerance)
    .value("PlannerDoubleParam_MinTimeResidualImprovement", PlannerDoubleParam_MinTimeResidualImprovement)
    .value("PlannerDoubleParam_WeightArmTorque", PlannerDoubleParam_WeightArmTorque)
	.value("PlannerDoubleParam_WeightLegTorque", PlannerDoubleParam_WeightLegTorque)
	.value("PlannerDoubleParam_WeightTimePenalty", PlannerDoubleParam_WeightTimePenalty)
	.value("PlannerDoubleParam_WeightTimeRegularization", PlannerDoubleParam_WeightTimeRegularization)
    .export_values();

  // binding string parameters
  py::enum_<PlannerStringParam>(m, "PlannerStringParam")
    .value("PlannerStringParam_ConfigFile", PlannerStringParam_ConfigFile)
    .value("PlannerStringParam_SaveDynamicsFile", PlannerStringParam_SaveDynamicsFile)
	.value("PlannerStringParam_DefaultSolverSettingFile", PlannerStringParam_DefaultSolverSettingFile)
    .export_values();

  // binding Eigen vector parameters
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
    .export_values();
}
