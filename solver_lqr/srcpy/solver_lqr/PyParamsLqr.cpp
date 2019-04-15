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

#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/SolverLqrParams.hpp>

namespace py = pybind11;
using namespace solverlqr;

void init_params_lqr(py::module &m)
{

  // binding integer parameters
  py::enum_<SolverLqrIntParam>(m, "SolverLqrIntParam")
    .value("SolverLqrIntParam_Verbosity", SolverLqrIntParam_Verbosity)
	.value("SolverLqrIntParam_TimeDimension", SolverLqrIntParam_TimeDimension)
	.value("SolverLqrIntParam_StateDimension", SolverLqrIntParam_StateDimension)
    .value("SolverLqrIntParam_PrecisionDigits", SolverLqrIntParam_PrecisionDigits)
    .value("SolverLqrIntParam_LqrMaxIterations", SolverLqrIntParam_LqrMaxIterations)
	.value("SolverLqrIntParam_ControlDimension", SolverLqrIntParam_ControlDimension)
	.value("SolverLqrIntParam_CurrentIteration", SolverLqrIntParam_CurrentIteration)
	.value("SolverLqrIntParam_BackpassDivergeIteration", SolverLqrIntParam_BackpassDivergeIteration)
	.value("SolverLqrIntParam_BackPassRegulatizationType", SolverLqrIntParam_BackPassRegulatizationType)
    .export_values();

  // binding boolean parameters
  py::enum_<SolverLqrBoolParam>(m, "SolverLqrBoolParam")
    .value("SolverLqrBoolParam_StoreData", SolverLqrBoolParam_StoreData)
	.value("SolverLqrBoolParam_HasControlLimits", SolverLqrBoolParam_HasControlLimits)
	.value("SolverLqrBoolParam_UseRungeKuttaIntegration", SolverLqrBoolParam_UseRungeKuttaIntegration)
    .export_values();

  // binding double parameters
  py::enum_<SolverLqrDoubleParam>(m, "SolverLqrDoubleParam")
	.value("SolverLqrDoubleParam_Cost", SolverLqrDoubleParam_Cost)
	.value("SolverLqrDoubleParam_TimeStep", SolverLqrDoubleParam_TimeStep)
	.value("SolverLqrDoubleParam_CostChange", SolverLqrDoubleParam_CostChange)
	.value("SolverLqrDoubleParam_TimeHorizon", SolverLqrDoubleParam_TimeHorizon)
	.value("SolverLqrDoubleParam_ExpectedCost", SolverLqrDoubleParam_ExpectedCost)
    .value("SolverLqrDoubleParam_ControlGradient", SolverLqrDoubleParam_ControlGradient)
	.value("SolverLqrDoubleParam_CurrentRegularization", SolverLqrDoubleParam_CurrentRegularization)
    .value("SolverLqrDoubleParam_CostChangeTolerance", SolverLqrDoubleParam_CostChangeTolerance)
    .value("SolverLqrDoubleParam_DivergenceLimitCheck", SolverLqrDoubleParam_DivergenceLimitCheck)
	.value("SolverLqrDoubleParam_ControlGradientTolerance", SolverLqrDoubleParam_ControlGradientTolerance)
	.value("SolverLqrDoubleParam_BackPassMinRegularization", SolverLqrDoubleParam_BackPassMinRegularization)
    .value("SolverLqrDoubleParam_BackPassMaxRegularization", SolverLqrDoubleParam_BackPassMaxRegularization)
	.value("SolverLqrDoubleParam_MinExpectedCostImprovement", SolverLqrDoubleParam_MinExpectedCostImprovement)
	.value("SolverLqrDoubleParam_BackPassInitialRegularization", SolverLqrDoubleParam_BackPassInitialRegularization)
	.value("SolverLqrDoubleParam_BackPassMultRegularizationIncr", SolverLqrDoubleParam_BackPassMultRegularizationIncr)
	.value("SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr", SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr)
    .export_values();

  // binding string parameters
  py::enum_<SolverLqrStringParam>(m, "SolverLqrStringParam")
    .value("SolverLqrStringParam_ConfigFile", SolverLqrStringParam_ConfigFile)
    .value("SolverLqrStringParam_SaveLqrFile", SolverLqrStringParam_SaveLqrFile)
    .export_values();

  // binding Eigen::VectorXd vector parameters
  py::enum_<SolverLqrVectorParam>(m, "SolverLqrVectorParam")
    .value("SolverLqrVectorParam_InitialState", SolverLqrVectorParam_InitialState)
    .value("SolverLqrVectorParam_MinControlLimits", SolverLqrVectorParam_MinControlLimits)
	.value("SolverLqrVectorParam_MaxControlLimits", SolverLqrVectorParam_MaxControlLimits)
	.value("SolverLqrVectorParam_LineSearchCoeffs", SolverLqrVectorParam_LineSearchCoeffs)
    .export_values();

  // binding yaml parameters
  py::enum_<SolverLqrYamlParam>(m, "SolverLqrYamlParam")
    .value("SolverLqrYamlParam_UserParameters", SolverLqrYamlParam_UserParameters)
    .export_values();

}
