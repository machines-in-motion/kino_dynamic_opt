/**
 * @file BnBSolver.hpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#pragma once

#include <solver/interface/Var.hpp>
#include <solver/optimizer/IPSolver.hpp>

namespace solver {

  enum class Decision { One = 1, Zero = 0, Undefined = -1 };
  enum class Status { NotSolved, SolvedBranchable, SolvedNonBranchable };

  struct node {
    Status status_;
    int partition_id_;
    double lower_bound_, upper_bound_, partition_val_;
  };

  class BnBSolver
  {
    public:
      BnBSolver(){}
      ~BnBSolver(){}

      ExitCode optimize();
      OptimizationVector& optimalVector() { return opt_; }
      void initialize(InteriorPointSolver& interior_point_solver,
                      const std::vector< std::shared_ptr<Var> >& binary_variables);

    private:
      ExitCode exitcode();
      void loadSolution();
      void storeSolution();
      void initializeRootNode();
      int selectNodeToExplore();
      double getProblemLowerBound();
      int optimilityCheck(int node_id);
      void createBranches(int node_id);
      void updateNodeBounds(int node_id);
      void selectPartitionVariable(int& partition_id, double& partition_val);
      void updateProblemData(const Eigen::Ref<const Eigen::VectorXi>& bool_node_id);

      inline OptimizationInfo& getInfo() { return optimization_info_; }
      inline InteriorPointSolver& getSolver() { return *interior_point_solver_; }

    private:
      OptimizationInfo optimization_info_;
      InteriorPointSolver* interior_point_solver_;

      ExitCode opt_status_;
      OptimizationVector opt_;
      std::vector<node> nodes_;
      Eigen::VectorXi binvars_ids_;
      Eigen::VectorXi binvars_vec_id_;
      Eigen::MatrixXi binvars_mat_id_;
      int nbin_vars_, iteration_, node_id_;
      double prob_upper_bound_, prob_lower_bound_;
  };
}
