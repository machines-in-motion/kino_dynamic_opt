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

#pragma once

namespace momentumopt {

  /*! Available integer variables used by the planner */
  enum PlannerIntParam {
    // Planner parameters
    PlannerIntParam_NumDofs,
	PlannerIntParam_NumTimesteps,
	PlannerIntParam_NumViapoints,
	PlannerIntParam_MaxNumTimeIterations,
	PlannerIntParam_NumActiveEndeffectors,
  };

  /*! Available boolean variables used by the planner */
  enum PlannerBoolParam {
    // Planner parameters
	PlannerBoolParam_IsTimeHorizonFixed,
	PlannerBoolParam_IsFrictionConeLinear,

	// Storage information
	PlannerBoolParam_StoreData,

	// Solver setting
	PlannerBoolParam_UseDefaultSolverSetting,
  };

  /*! Available double variables used by the planner */
  enum PlannerDoubleParam {
    // Planner parameters
	PlannerDoubleParam_Gravity,
	PlannerDoubleParam_TimeStep,
	PlannerDoubleParam_RobotMass,
	PlannerDoubleParam_RobotWeight,
	PlannerDoubleParam_TimeHorizon,
	PlannerDoubleParam_MassTimesGravity,
	PlannerDoubleParam_FrictionCoefficient,
	PlannerDoubleParam_MaxTimeResidualTolerance,
	PlannerDoubleParam_MinTimeResidualImprovement,

	// Optimization weights
    PlannerDoubleParam_WeightArmTorque,
	PlannerDoubleParam_WeightLegTorque,
    PlannerDoubleParam_WeightTimePenalty,
	PlannerDoubleParam_WeightTimeRegularization,
  };

  /*! Available string variables used by the planner */
  enum PlannerStringParam {
    // Storage information
    PlannerStringParam_ConfigFile,
	PlannerStringParam_SaveDynamicsFile,

	// Solver setting
	PlannerStringParam_DefaultSolverSettingFile,
  };

  /*! Available vector variables used by the planner */
  enum PlannerIntVectorParam {
  };

  /*! Available vector variables used by the planner */
  enum PlannerVectorParam {
	// Dynamics parameters
	PlannerVectorParam_TimeRange,
	PlannerVectorParam_TorqueRange,
	PlannerVectorParam_ExternalForce,
	PlannerVectorParam_GravityVector,
	PlannerVectorParam_CenterOfMassMotion,
	PlannerVectorParam_MaxEndeffectorLengths,

	// Dynamics optimization weights
    PlannerVectorParam_WeightArmForce,
	PlannerVectorParam_WeightLegForce,
	PlannerVectorParam_WeightArmForceRate,
	PlannerVectorParam_WeightLegForceRate,
	PlannerVectorParam_WeightCenterOfMass,
	PlannerVectorParam_WeightLinearMomentum,
	PlannerVectorParam_WeightAngularMomentum,
    PlannerVectorParam_WeightLinearMomentumRate,
    PlannerVectorParam_WeightAngularMomentumRate,
	PlannerVectorParam_WeightFinalLinearMomentum,
	PlannerVectorParam_WeightCenterOfMassViapoint,
    PlannerVectorParam_WeightFinalAngularMomentum,
	PlannerVectorParam_WeightDynamicTrackingLinearMomentum,
	PlannerVectorParam_WeightDynamicTrackingAngularMomentum,
  };

  /*! Available array variables used by the planner */
  enum PlannerArrayParam {
    PlannerArrayParam_EndeffectorOffset,
    PlannerArrayParam_CenterOfPressureRange,
  };

  /*! Available c-vector variables used by the planner */
  enum PlannerCVectorParam {
    PlannerCVectorParam_Viapoints,
  };

}
