/**
 * @file SolverSetting.hpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#pragma once

#include <string>
#include <limits>
#include <solver/interface/SolverParams.hpp>

namespace solver {

  enum class ExitCode {
    Optimal,           /*! Problem solved to optimality                           */
    OptimalInacc,      /*! Problem solved to optimality (Inaccurate)              */
    PrimalInf,         /*! Found certificate of primal infeasibility              */
    PrimalInfInacc,    /*! Found certificate of primal infeasibility (Inaccurate) */
    DualInf,           /*! Found certificate of dual infeasibility                */
    DualInfInacc,      /*! Found certificate of dual infeasibility (Inaccurate)   */
	ReachMaxIters,     /*! Algorithm reached maximum number of iterations         */
    NotConverged,      /*! Algorithm has not converged yet                        */
    Indeterminate,     /*! Nothing can be said about the solution                 */
    PrSearchDirection, /*! Not reliable search direction                          */
	PrSlacksLeaveCone, /*! Slack variables lie outside convex cone                */
	PrProjection,      /*! Failed in the projection of cone or linear system      */
  };

  enum class ConeStatus { Inside, Outside };
  enum class FactStatus { Optimal, Failure };
  enum class PrecisionConvergence { Full, Reduced };
  enum class QuadConstrApprox { None, TrustRegion, SoftConstraint  };

  /**
   * Class that provides access to all environment variables required by the solver
   */
  class SolverSetting
  {
    public:
	  SolverSetting(){}
	  ~SolverSetting(){}

	  void initialize(const std::string cfg_file, const std::string solver_vars_yaml = "solver_variables");

      int get(SolverIntParam param) const;
      bool get(SolverBoolParam param) const;
      double get(SolverDoubleParam param) const;

      void set(SolverIntParam param, int value);
      void set(SolverBoolParam param, bool value);
      void set(SolverDoubleParam param, double value);

	  static constexpr double nan = ((double)0x7ff8000000000000);
	  static constexpr double inf = ((double)std::numeric_limits<double>::infinity());

    private:
	  // Branch and Bound solver
	  bool BnB_verbose_;
	  int  BnB_max_iterations_;
	  double BnB_integer_tolerance_, BnB_absolute_suboptimality_gap_, BnB_relative_suboptimality_gap_;

	  // Convergence tolerances
	  double feasibility_tolerance_, absolute_suboptimality_gap_, relative_suboptimality_gap_,
	         feasibility_tolerance_inaccurate_, absolute_suboptimality_gap_inaccurate_, relative_suboptimality_gap_inaccurate_;

	  // Equilibration parameters
	  int equil_iterations_;

	  // Linear System parameters
	  int num_iter_ref_lin_solve_;
	  double dyn_reg_thresh_, lin_sys_accuracy_, err_reduction_factor_, static_regularization_, dynamic_regularization_;

	  // Algorithm parameters
	  double safeguard_, min_step_length_, max_step_length_, min_centering_step_, max_centering_step_, step_length_scaling_;

	  // Model parameters
	  bool verbose_;
	  double trust_region_threshold_, soft_constraint_weight_full_, soft_constraint_weight_reduced_;
	  int max_iters_, num_itrefs_trustregion_, ipsolver_warm_iters_, ipsolver_max_iters_;
  };

}
