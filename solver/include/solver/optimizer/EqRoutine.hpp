/**
 * @file EqRoutine.hpp
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
#include <Eigen/Sparse>
#include <solver/interface/Cone.hpp>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  /*! Equilibration routine to improve condition number of
   *  matrices involved in the optimization problem. The method
   *  provided by default is Ruiz equilibration.
   */
  class EqRoutine
  {
    public:
	  EqRoutine(){}
	  ~EqRoutine(){}

	  void setEquilibration(const Cone& cone, const SolverSetting& stgs, SolverStorage& stg);
	  void unsetEquilibration(SolverStorage& stg);
	  void scaleVariables(OptimizationVector& opt);

      Vector& equilVec() { return equil_vec_; }
      const Vector& equilVec() const { return equil_vec_; }

    private:
	  void ruizEquilibration(SolverStorage& stg);
	  void maxRowsCols(double *row_vec, double *col_vec, const Eigen::SparseMatrix<double>& mat);
	  void equilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat);
	  void unequilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat);

    private:
	  Vector equil_vec_;
	  std::shared_ptr<Cone> cone_;
	  std::shared_ptr<SolverSetting> stgs_;
  };

}
