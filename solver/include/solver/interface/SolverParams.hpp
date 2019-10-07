/**
 * @file SolverParams.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

namespace solver {

  /*! Available integer variables used by the optimizer */
  enum SolverIntParam {
	// Branch and Bound solver
	SolverIntParam_BnBMaxIters,

	// Equilibration parameters
	SolverIntParam_EquilibrationIters,

	// Linear System parameters
	SolverIntParam_NumIterRefinementsLinSolve,

	// Model parameters
	SolverIntParam_MaxIters,
    SolverIntParam_WarmStartIters,
    SolverIntParam_SolverMaxIters,
	SolverIntParam_NumberRefinementsTrustRegion,

	// Variable parameters
    SolverIntParam_ColNum,

	// OptimizationInfo parameters
	SolverIntParam_NumIter,
	SolverIntParam_NumRefsLinSolve,
	SolverIntParam_NumRefsLinSolveAffine,
	SolverIntParam_NumRefsLinSolveCorrector,
  };

  /*! Available boolean variables used by the optimizer */
  enum SolverBoolParam {
	SolverBoolParam_Verbose,
	SolverBoolParam_BnBVerbose
  };

  /*! Available double variables used by the optimizer */
  enum SolverDoubleParam {
	// Branch and Bound solver
	SolverDoubleParam_BnBIntegerTol,
	SolverDoubleParam_BnBAbsSubOptGap,
	SolverDoubleParam_BnBRelSubOptGap,

	// Convergence tolerances
	SolverDoubleParam_FeasibilityTol,
	SolverDoubleParam_DualityGapAbsTol,
	SolverDoubleParam_DualityGapRelTol,
	SolverDoubleParam_FeasibilityTolInacc,
	SolverDoubleParam_DualityGapAbsTolInacc,
	SolverDoubleParam_DualityGapRelTolInacc,

	// Linear System parameters
	SolverDoubleParam_LinearSystemAccuracy,
	SolverDoubleParam_ErrorReductionFactor,
	SolverDoubleParam_StaticRegularization,
	SolverDoubleParam_DynamicRegularization,
	SolverDoubleParam_DynamicRegularizationThresh,

	// Algorithm parameters
	SolverDoubleParam_SafeGuard,
	SolverDoubleParam_MinimumStepLength,
	SolverDoubleParam_MaximumStepLength,
	SolverDoubleParam_StepLengthScaling,
	SolverDoubleParam_MinimumCenteringStep,
	SolverDoubleParam_MaximumCenteringStep,

	// Model parameters
	SolverDoubleParam_TrustRegionThreshold,
	SolverDoubleParam_SoftConstraintWeightFull,
	SolverDoubleParam_SoftConstraintWeightReduced,

	// Variable parameters
    SolverDoubleParam_X,
    SolverDoubleParam_LB,
    SolverDoubleParam_UB,
    SolverDoubleParam_Guess,

	// OptimizationInfo parameters
	SolverDoubleParam_Tau,
	SolverDoubleParam_Kappa,
	SolverDoubleParam_DualCost,
	SolverDoubleParam_PrimalCost,
	SolverDoubleParam_DualityGap,
	SolverDoubleParam_KappaOverTau,
	SolverDoubleParam_DualResidual,
	SolverDoubleParam_MeritFunction,
	SolverDoubleParam_PrimalResidual,
	SolverDoubleParam_DualInfeasibility,
	SolverDoubleParam_RelativeDualityGap,
	SolverDoubleParam_PrimalInfeasibility,

	SolverDoubleParam_StepLength,
	SolverDoubleParam_AffineStepLength,
	SolverDoubleParam_CorrectionStepLength,
  };

}
