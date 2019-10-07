/**
 * @file NcvxBnBSolver.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <solver/interface/Var.hpp>
#include <solver/optimizer/IPSolver.hpp>
#include <solver/optimizer/BnBSolver.hpp>
#include <solver/interface/ConicProblem.hpp>

namespace solver {

  class NcvxBnBSolver
  {
    public:
      NcvxBnBSolver(){}
      ~NcvxBnBSolver(){}

      ExitCode optimize();
      void initialize(ConicProblem& conic_problem);
      OptimizationVector& optimalVector() { return opt_; }

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

      inline ConicProblem& getProblem() { return *conic_problem_; }
      inline OptimizationInfo& getInfo() { return optimization_info_; }

    private:
      ConicProblem* conic_problem_;
      OptimizationInfo optimization_info_;

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
