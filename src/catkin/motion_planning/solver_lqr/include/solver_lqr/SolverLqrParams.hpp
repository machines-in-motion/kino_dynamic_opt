#pragma once

namespace solverlqr {

  /*! Available integer variables used by the optimizer */
  enum SolverLqrIntParam {
	// LQR-like algorithm parameters
	SolverLqrIntParam_Verbosity,
	SolverLqrIntParam_PrecisionDigits,
	SolverLqrIntParam_LqrMaxIterations,
	SolverLqrIntParam_BackPassRegulatizationType,

	// LQR problem parameters
	SolverLqrIntParam_TimeDimension,
	SolverLqrIntParam_StateDimension,
	SolverLqrIntParam_ControlDimension,

	// LQR Optimization Info parameters
	SolverLqrIntParam_CurrentIteration,
	SolverLqrIntParam_BackpassDivergeIteration,
  };

  /*! Available boolean variables used by the optimizer */
  enum SolverLqrBoolParam {
	// LQR-like algorithm parameters
	SolverLqrBoolParam_StoreData,

	// LQR problem parameters
	SolverLqrBoolParam_HasControlLimits,
	SolverLqrBoolParam_UseRungeKuttaIntegration,
  };

  /*! Available double variables used by the optimizer */
  enum SolverLqrDoubleParam {
	// LQR-like algorithm parameters
	SolverLqrDoubleParam_CostChangeTolerance,
	SolverLqrDoubleParam_DivergenceLimitCheck,
	SolverLqrDoubleParam_ControlGradientTolerance,
	SolverLqrDoubleParam_BackPassMinRegularization,
	SolverLqrDoubleParam_BackPassMaxRegularization,
	SolverLqrDoubleParam_MinExpectedCostImprovement,
	SolverLqrDoubleParam_BackPassInitialRegularization,
	SolverLqrDoubleParam_BackPassMultRegularizationIncr,
	SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr,

	// LQR problem parameters
	SolverLqrDoubleParam_TimeStep,
	SolverLqrDoubleParam_TimeHorizon,

	// LQR Optimization Info parameters
	SolverLqrDoubleParam_Cost,
	SolverLqrDoubleParam_CostChange,
	SolverLqrDoubleParam_ExpectedCost,
	SolverLqrDoubleParam_ControlGradient,
	SolverLqrDoubleParam_CurrentRegularization,
  };

  /*! Available string variables used by the optimizer */
  enum SolverLqrStringParam {
    // Storage information
    SolverLqrStringParam_ConfigFile,
    SolverLqrStringParam_SaveLqrFile,
  };

  /*! Available vector variables used by the optimizer */
  enum SolverLqrVectorParam {
	// LQR-like algorithm parameters
	SolverLqrVectorParam_LineSearchCoeffs,

	// LQR problem parameters
	SolverLqrVectorParam_InitialState,
	SolverLqrVectorParam_MinControlLimits,
	SolverLqrVectorParam_MaxControlLimits,
  };

  /*! Available yaml variables used by the optimizer */
  enum SolverLqrYamlParam {
	// LQR problem parameters
    SolverLqrYamlParam_UserParameters,
  };

}
