/**
 * @file LinSolver.hpp
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
#include <solver/interface/SolverSetting.hpp>
#include <solver/optimizer/SparseCholesky.hpp>

namespace solver {

  /**
   * Class that provides functionality for handling solution of linear systems
   * in optimization problems. Performs symbolic and numeric factorization of
   * kkt matrix, find permutation of kkt matrix to induce the best possible
   * sparsity pattern, builds and updates kkt matrix and its scalings as required.
   */
  class LinSolver
  {
    public:
      LinSolver(){}
      ~LinSolver(){}

      void updateMatrix();
      void initializeMatrix();
      FactStatus numericFactorization();
      void initialize(Cone& cone, SolverSetting& stgs, SolverStorage& stg);
      int solve(const Eigen::Ref<const Eigen::VectorXd>& permB, OptimizationVector& searchDir, bool is_initialization = false);
      void matrixTransposeTimesVector(const Eigen::SparseMatrix<double>& A,const Eigen::Ref<const Eigen::VectorXd>& eig_x, Eigen::Ref<Eigen::VectorXd> eig_y, bool add = true, bool is_new = true);

      // Some getter and setter methods
      int perm(int id) { return perm_.indices()[id]; }
      int invPerm(int id) { return invPerm_.indices()[id]; }
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& perm() { return perm_; }
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& invPerm() { return invPerm_; }
      const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& perm() const { return perm_; }
      const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& invPerm() const { return invPerm_; }

    private:
      inline Cone& getCone() { return *cone_; }
      inline SolverStorage& getStorage() { return *storage_; }
      inline SolverSetting& getSetting() { return *setting_; }
      inline linalg::SparseCholesky& getCholesky() { return cholesky_; }

      void buildProblem();
      void findPermutation();
      void resizeProblemData();
      void symbolicFactorization();

    private:
      Cone* cone_;
      SolverStorage* storage_;
      SolverSetting* setting_;
      linalg::SparseCholesky cholesky_;

      ConicVector Gdx_;
      double static_regularization_;
      ExtendedVector sign_, permSign_;
      Eigen::VectorXd permX_, Pe_, permdX_;
      Eigen::SparseMatrix<double> kkt_, permKkt_;
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm_, invPerm_, permK_;
  };
}
