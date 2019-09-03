/*
 * Copyright [2019] Max Planck Society. All rights reserved.
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

#include <boost/thread.hpp>
#include <solver/interface/Exprs.hpp>
#include <solver/rt_optimizer/RtHQPCost.hpp>
#include <solver/rt_optimizer/RtHQPSolver.hpp>

namespace rt_solver {

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  class RtModel
  {
    public:
      RtModel(YAML::Node params);
      virtual ~RtModel(){}

      bool solve();
      void initialize();
      virtual void update(){}

      // helper functions
      template <typename VecDer>
      double inequalitySlack(Eigen::MatrixBase<VecDer>& dist);
      template <typename Mat>
      void reduceColumns(Eigen::EigenBase<Mat>& mat) const;
      template<int Max_Rows, typename Mat_Type, typename Vec_Type>
      void appendRowsOfRank(int rank_to_add, const typename rt_solver::RtVector<Max_Rows>::i& ranks,
        const Eigen::MatrixBase<Mat_Type>& subst_mat, const Eigen::MatrixBase<Vec_Type>& subst_vec, bool append_to_equalities, int starting_column);

      // getter and setter methods
      typename rt_solver::RtVector<Max_Eq_Rows>::d& nextEqCostVec() { return next_eq_cost_vec_; }
      typename rt_solver::RtVector<Max_Ineq_Rows>::d& nextIneqCostVec() { return next_ineq_cost_vec_; }
      typename rt_solver::RtMatrix<Max_Eq_Rows, Num_OptVars>::d& nextEqCostMat() { return next_eq_cost_mat_; }
      typename rt_solver::RtMatrix<Max_Ineq_Rows, Num_OptVars>::d& nextIneqCostMat() { return next_ineq_cost_mat_; }

      const typename rt_solver::RtVector<Max_Eq_Rows>::d& nextEqCostVec() const { return next_eq_cost_vec_; }
      const typename rt_solver::RtVector<Max_Ineq_Rows>::d& nextIneqCostVec() const { return next_ineq_cost_vec_; }
      const typename rt_solver::RtMatrix<Max_Eq_Rows, Num_OptVars>::d& nextEqCostMat() const { return next_eq_cost_mat_; }
      const typename rt_solver::RtMatrix<Max_Ineq_Rows, Num_OptVars>::d& nextIneqCostMat() const { return next_ineq_cost_mat_; }

      std::vector<rt_solver::RtHQPCost*>& subCostComposers() { return sub_cost_composers_; }

      // methods to use easier interface to formulate problems
      void clean();
      solver::Var getVar(const int var_id);
      bool checkIfHigherRanksLeft(const int rank);
      void checkSlacknessOfHierarchy(const int rank);
      void addCostsToHierarchy(const int rank_to_add);
      void addLinConstr(const solver::LinExpr& lhs, const std::string sense, const solver::LinExpr& rhs, const int rank);
      void print();

      std::vector<int>& hierarchiesWithoutSlacks() { return hierarchies_without_slacks_; }
      const std::vector<int>& hierarchiesWithoutSlacks() const { return hierarchies_without_slacks_; }

    public:
      double n_solved_ranks_;
      bool is_solution_valid_;
      int num_variables_optimized_;
      std::vector<double> qp_dur_, svd_wait_;
      Eigen::Matrix<int, Num_OptVars, 1> full_to_opt_variable_index_;
      mutable rt_solver::RtHQPSolver<Max_Ineq_Rows, Num_OptVars, Max_Eq_Rows> hqp_solver_;

    private:
      std::vector<solver::Var> vars_;
      std::vector<int> hierarchies_without_slacks_;
      std::vector<std::tuple<int, solver::LinExpr>> leqcons_, lineqcons_;

      std::vector<rt_solver::RtHQPCost*> sub_cost_composers_;
      typename rt_solver::RtVector<Max_Eq_Rows>::d next_eq_cost_vec_;
      typename rt_solver::RtVector<Max_Ineq_Rows>::d next_ineq_cost_vec_;
      typename rt_solver::RtMatrix<Max_Eq_Rows, Num_OptVars>::d next_eq_cost_mat_;
      typename rt_solver::RtMatrix<Max_Ineq_Rows, Num_OptVars>::d next_ineq_cost_mat_;

  };

}
