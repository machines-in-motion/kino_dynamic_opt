/*
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
