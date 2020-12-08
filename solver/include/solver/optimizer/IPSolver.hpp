/**
 * @file IPSolver.hpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#pragma once

#include <solver/interface/Cone.hpp>
#include <solver/optimizer/LinSolver.hpp>
#include <solver/optimizer/EqRoutine.hpp>
#include <solver/interface/SolverSetting.hpp>
#include <solver/optimizer/CvxInfoPrinter.hpp>

namespace solver {

  class Model;
  class BnBSolver;
  class NcvxBnBSolver;

  /**
   * Main class that implements an Interior Point Solver for Second-Order Cones.
   * Details can be found in the paper: Domahidi, A. and Chu, E. and Boyd, S.,
   *     ECOS: An SOCP solver for embedded systems, ECC 2013, pages 3071-3076
   */
  class InteriorPointSolver
  {
    public:
      InteriorPointSolver(){}
      ~InteriorPointSolver() {}

      ExitCode optimize();
      const OptimizationVector& optimalVector() const { return opt_; }
      void initialize(SolverStorage& stg, Cone& cone, SolverSetting& stgs);
      int current_iter;

    private:
      inline Cone& getCone() { return *cone_; }
      inline SolverStorage& getStorage() { return *storage_; }
      inline SolverSetting& getSetting() { return *setting_; }
      inline CvxInfoPrinter& getPrinter() { return printer_; }
      inline OptimizationVector& optimalVector() { return opt_; }
      inline LinSolver& getLinSolver() { return linear_solver_; }
      inline OptimizationInfo& getInfo() { return optimization_info_; }
      inline EqRoutine& getEqRoutine() { return equilibration_routine_; }
      inline OptimizationInfo& getBestInfo() { return best_optimization_info_; }

      void rhsAffineStep();
      void computeResiduals();
      void updateStatistics();
      void saveIterateAsBest();
      void restoreBestIterate();
      void internalInitialization();
      ExitCode initializeVariables();
      void rhsCenteringPredictorStep();
      void updateEquilH(int id, double value);
      ExitCode convergenceCheck(const PrecisionConvergence& mode);
      double lineSearch(const Eigen::Ref<const Eigen::VectorXd>& dsvec,
                        const Eigen::Ref<const Eigen::VectorXd>& dzvec,
                        double tau, double dtau, double kappa, double dkappa);

      friend class Model;
      friend class BnBSolver;
      friend class ConicProblem;
      friend class NcvxBnBSolver;

    private:
      Vector res_;
      ExitCode exitcode_;
      ExtendedVector rhs1_, rhs2_;
      OptimizationVector opt_, best_opt_, dopt1_, dopt2_;
      ConicVector lambda_, rho_, sigma_, lbar_, ds_affine_by_W_, W_times_dz_affine_, ds_combined_, dz_combined_;
      double dk_combined_, dt_affine_, dk_affine_, inires_x_, inires_y_, inires_z_, dt_denom_,
             residual_t_, residual_x_, residual_y_, residual_z_, cx_, by_, hz_, prev_pres_;

      Cone* cone_;
      SolverStorage* storage_;
      SolverSetting* setting_;
      CvxInfoPrinter printer_;
      LinSolver linear_solver_;
      EqRoutine equilibration_routine_;
      OptimizationInfo optimization_info_, best_optimization_info_;
  };
}
