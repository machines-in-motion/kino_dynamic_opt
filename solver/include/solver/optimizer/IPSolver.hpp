/*
 *
 * ECOS - Embedded Conic Solver
 * Copyright (C) [2012-2015] A. Domahidi [domahidi@embotech.com],
 * Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 *
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
