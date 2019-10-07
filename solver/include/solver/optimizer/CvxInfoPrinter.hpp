/**
 * @file CvxInfoPrinter.hpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#pragma once

#include <memory>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  enum class Msg {
    MatrixFactorization,    /*! Problem in matrix factorization           */
    SearchDirection,        /*! Unreliable search direction               */
    NumericalProblem,       /*! Numerical problems                        */
    LineSearchStagnation,   /*! Line search step too small                */
    VariablesLeavingCone,   /*! Slacks or multipliers leaving cone        */
    MaxItersReached,        /*! Maximum number of iterations reached      */
    OptimalityReached,      /*! Problem solved to optimality              */
    PrimalInfeasibility,    /*! Certificate of primal infeasibility found */
    DualInfeasibility,      /*! Certificate of dual infeasibility found   */
    OptimizationProgress,   /*! Progress of current optimization iterate  */
  };

  /**
   * Helper class that contains information about the status of the optimization problem
   */
  class OptimizationInfo
  {
    public:
      OptimizationInfo() : iteration_(-1){}
      ~OptimizationInfo(){}

      bool isBetterThan(const OptimizationInfo& info) const;
      OptimizationInfo& operator=(const OptimizationInfo& other);

      int& get(const SolverIntParam& param);
      double& get(const SolverDoubleParam& param);
      PrecisionConvergence& mode() { return mode_; }

      const int& get(const SolverIntParam& param) const;
      const double& get(const SolverDoubleParam& param) const;
      const PrecisionConvergence& mode() const { return mode_; }

    private:
      PrecisionConvergence mode_;
      int iteration_, linear_solve_refinements_, affine_linear_solve_refinements_, correction_linear_solve_refinements_;
      double primal_cost_, dual_cost_, primal_residual_, dual_residual_, primal_infeasibility_, dual_infeasibility_,
             tau_, kappa_, kappa_over_tau_, merit_function_, duality_gap_, relative_duality_gap_, correction_step_length_,
             step_length_, affine_step_length;
  };

  class CvxInfoPrinter
  {
    public:
      CvxInfoPrinter(){}
      ~CvxInfoPrinter(){}

      void initialize(const SolverSetting& stgs) { stgs_ = std::make_shared<SolverSetting>(stgs); }
      void display(const Msg& msg, const OptimizationInfo& info);

    private:
      std::shared_ptr<SolverSetting> stgs_;
  };
}
