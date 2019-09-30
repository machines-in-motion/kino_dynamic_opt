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

#include <solver/rt_optimizer/RtModel.hpp>
#include <solver/rt_optimizer/RtHQPSolver.h>

namespace rt_solver {

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::RtModel(YAML::Node params)
    : hqp_solver_(params)
  {
    is_solution_valid_ = false;
    num_variables_optimized_ = Num_OptVars;

    for (int var_id=0; var_id<Num_OptVars; var_id++) {
      full_to_opt_variable_index_[var_id] = var_id;
      vars_.push_back( solver::Var(vars_.size(), solver::VarType::Continuous, 0.0, 1.0, 0.0) );
    }
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::initialize()
  {
    qp_dur_.resize(5);
    svd_wait_.resize(5);
    hqp_solver_.initialize();
    is_solution_valid_ = false;
    hierarchies_without_slacks_.clear();
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  template <typename Mat>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::reduceColumns(Eigen::EigenBase<Mat>& mat) const
  {
    #ifndef RTEIG_NO_ASSERTS
      assert(mat.cols() == full_to_opt_variable_index_.size());
    #endif
    unsigned int first_empty_i=0;
    for (; first_empty_i < full_to_opt_variable_index_.size(); ++first_empty_i) {
      if (full_to_opt_variable_index_[first_empty_i] < 0)
        break;
    }

    //found an empty row, now find the next nonempty row
    unsigned int block_start_i=first_empty_i+1;
    while (true) {
      for (; block_start_i < full_to_opt_variable_index_.size(); ++block_start_i)
        if (full_to_opt_variable_index_[block_start_i] >= 0)
          break;

      // if the rest of the matrix is zero, stop
      if (block_start_i >= full_to_opt_variable_index_.size())
        break;

      // now find the end of the block
      unsigned int block_end_i=block_start_i+1;
      for (; block_end_i < full_to_opt_variable_index_.size(); ++block_end_i)
        if (full_to_opt_variable_index_[block_end_i] < 0)
          break;

      //now shift the block up
      mat.derived().block(0, first_empty_i, mat.rows(),block_end_i-block_start_i) =
      mat.derived().block(0, block_start_i, mat.rows(), block_end_i-block_start_i);

      first_empty_i += block_end_i-block_start_i;
      block_start_i = block_end_i;
    }
    rt_solver::RtMatrixUtils::conservativeResize(mat.derived(), mat.rows(), first_empty_i);
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  template<int Max_Rows, typename Mat_Type, typename Vec_Type>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::appendRowsOfRank(
    int rank_to_add, const typename rt_solver::RtVector<Max_Rows>::i& ranks,
    const Eigen::MatrixBase<Mat_Type>& subst_mat, const Eigen::MatrixBase<Vec_Type>& subst_vec, bool append_to_equalities, int starting_column)
  {
    // count how many rows we add in this rank
    int n_add_rows = 0;
    for (int i=0; i<ranks.size(); ++i)
      if (ranks[i] == rank_to_add)
        ++n_add_rows;

    if (0 == n_add_rows) { return; } // nothing to be done
    else {
      //extend the problem matrix by the amount of rows we need
      int old_rows;
      if (append_to_equalities)
      {
        old_rows = next_eq_cost_mat_.rows();
        rt_solver::RtVectorUtils::conservativeResize(next_eq_cost_vec_, old_rows+n_add_rows);
        rt_solver::RtMatrixUtils::conservativeResize(next_eq_cost_mat_, old_rows+n_add_rows, next_eq_cost_mat_.cols());
        next_eq_cost_mat_.bottomRows(n_add_rows).setZero();
        next_eq_cost_vec_.bottomRows(n_add_rows).setZero();
      } else {
        old_rows = next_ineq_cost_mat_.rows();
        rt_solver::RtVectorUtils::conservativeResize(next_ineq_cost_vec_, old_rows+n_add_rows);
        rt_solver::RtMatrixUtils::conservativeResize(next_ineq_cost_mat_, old_rows+n_add_rows, next_ineq_cost_mat_.cols());
        next_ineq_cost_mat_.bottomRows(n_add_rows).setZero();
        next_ineq_cost_vec_.bottomRows(n_add_rows).setZero();
      }

      // add the rows from mat that correspond with the requested rank
      int added_row = 0;
      for (int i=0; i<ranks.size(); ++i) {
        if (ranks[i] == rank_to_add) {
          if (append_to_equalities) {
            next_eq_cost_mat_.block(old_rows+added_row, starting_column, 1, subst_mat.cols()) = subst_mat.row(i);
            next_eq_cost_vec_(old_rows+added_row) = subst_vec(i);
          } else {
            next_ineq_cost_mat_.block(old_rows+added_row, starting_column, 1, subst_mat.cols()) = subst_mat.row(i);
            next_ineq_cost_vec_(old_rows+added_row) = subst_vec(i);
          }
          ++added_row;
        }
      }
    }
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  template <typename VecDer>
  double RtModel<  Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::inequalitySlack(Eigen::MatrixBase<VecDer>& dist) {
    #ifndef RTEIG_NO_ASSERTS
      assert(dist.cols() == 1 && "dist has to be a vector");
    #endif
    for (int i=0;i<dist.rows();++i) { dist[i]<=0.0 ? dist[i]*=0.0 : dist[i]*=1.0; }
    return dist.norm();
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  bool RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::solve()
  {
    n_solved_ranks_ = 0.0;
    is_solution_valid_ = true;
    bool higher_ranks_left = true;
    bool problem_dofs_left = true;

    for (int rank = 0; higher_ranks_left && problem_dofs_left && is_solution_valid_; rank++)
    {
      // reset task costs
      rt_solver::RtVectorUtils::resize(next_eq_cost_vec_, 0);
      rt_solver::RtVectorUtils::resize(next_ineq_cost_vec_, 0);
      rt_solver::RtMatrixUtils::resize(next_eq_cost_mat_, 0, Num_OptVars);
      rt_solver::RtMatrixUtils::resize(next_ineq_cost_mat_, 0, Num_OptVars);

      // check if different slackness properties have been specified for hierarchies
      if (hierarchies_without_slacks_.size()>0)
        this->checkSlacknessOfHierarchy(rank);

      // ask composers of this rank to add their sub-costs
      higher_ranks_left = this->checkIfHigherRanksLeft(rank);
      for (unsigned int cst_id=0; cst_id<sub_cost_composers_.size(); cst_id++) {
        sub_cost_composers_[cst_id]->addCostToHierarchy(rank);
        if (sub_cost_composers_[cst_id]->maxRank() > rank)
          higher_ranks_left = true;
      }
      this->addCostsToHierarchy(rank);

      for (int r=0; r<next_eq_cost_mat_.rows(); r++) {
        if (!next_eq_cost_mat_.row(r).allFinite() || !std::isfinite(next_eq_cost_vec_[r])) {
          is_solution_valid_ = false;
          std::cout << "bad eq in rank " << rank << ", row: " << r << std::endl;
        }
      }

      for (int r=0; r<next_ineq_cost_mat_.rows(); r++) {
        if (!next_ineq_cost_mat_.row(r).allFinite() || !std::isfinite(next_ineq_cost_vec_[r])) {
          is_solution_valid_ = false;
          std::cout << "bad ineq in rank " << rank << ", row: " << r << std::endl;
        }
      }

      reduceColumns(next_eq_cost_mat_);
      reduceColumns(next_ineq_cost_mat_);
      rt_solver::RtAffineUtils::removeZeroRows(next_eq_cost_mat_, next_eq_cost_vec_);
      rt_solver::RtAffineUtils::removeZeroRows(next_ineq_cost_mat_, next_ineq_cost_vec_);

      // add sub costs that are already reduced internally
      for (unsigned int i = 0; i < sub_cost_composers_.size(); ++i) {
        sub_cost_composers_[i]->addCostToHierarchyAfterReduction(rank);
        if (sub_cost_composers_[i]->maxRank() > rank)
          higher_ranks_left = true;
      }

      if (rank == 0) { hqp_solver_.reset(num_variables_optimized_); }

      if (hqp_solver_.solveNextTask(next_eq_cost_mat_, next_eq_cost_vec_,
                                    next_ineq_cost_mat_, next_ineq_cost_vec_)) {
        n_solved_ranks_ += 1;
      } else { is_solution_valid_ = false; }

      if (rank < int(qp_dur_.size()) && rank < int(svd_wait_.size()) ) {
        qp_dur_[rank] = hqp_solver_.qpSolveDuration();
        svd_wait_[rank] = hqp_solver_.qpWaitDuration();
      }
      problem_dofs_left = (hqp_solver_.nullspaceDimension() != 0);
    }

    //notify sub-composers of new solution
    for (unsigned int i = 0; i < this->subCostComposers().size(); ++i)
      this->subCostComposers()[i]->updateAfterSolutionFound();

    return is_solution_valid_;
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::clean()
  {
    leqcons_.clear();
    lineqcons_.clear();
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  solver::Var RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::getVar(const int var_id)
  {
    assert(var_id < Num_OptVars && "VarId exceeds max number of optimization variables available");
    return vars_[var_id];
  }

  // Linear Constraint: left_hand_side [< = >] right_hand_side
  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::addLinConstr(const solver::LinExpr& lhs, const std::string sense, const solver::LinExpr& rhs, const int rank)
  {
    if (sense == "=") { leqcons_.push_back(std::make_tuple(rank, lhs-rhs)); }
    else if (sense == "<") { lineqcons_.push_back(std::make_tuple(rank, lhs-rhs)); }
    else if (sense == ">") { lineqcons_.push_back(std::make_tuple(rank, rhs-lhs)); }
    else { throw std::runtime_error("Invalid sense on Linear Constraint"); }
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  bool RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::checkIfHigherRanksLeft(const int rank)
  {
    for (size_t leq_id=0; leq_id<leqcons_.size(); leq_id++)
      if (std::get<0>(leqcons_[leq_id]) > rank) { return true; }
    for (size_t lineq_id=0; lineq_id<lineqcons_.size(); lineq_id++)
      if (std::get<0>(lineqcons_[lineq_id]) > rank) { return true; }

    return false;
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::addCostsToHierarchy(const int rank_to_add)
  {
    /*! appending linear equalities */
    // count how many rows we add in this rank
    int n_add_rows = 0;
    for (size_t row_id=0; row_id<leqcons_.size(); row_id++)
      if (std::get<0>(leqcons_[row_id]) == rank_to_add) { n_add_rows++; }

    if (n_add_rows > 0) {
      // extend the problem matrix by the amount of rows we need
      int old_rows = next_eq_cost_mat_.rows();
      rt_solver::RtVectorUtils::conservativeResize(next_eq_cost_vec_, old_rows+n_add_rows);
      rt_solver::RtMatrixUtils::conservativeResize(next_eq_cost_mat_, old_rows+n_add_rows, next_eq_cost_mat_.cols());
      next_eq_cost_mat_.bottomRows(n_add_rows).setZero();
      next_eq_cost_vec_.bottomRows(n_add_rows).setZero();

      // add the rows from mat that correspond with the requested rank
      int added_row = 0;
      for (size_t row_id=0; row_id<leqcons_.size(); row_id++)
        if (std::get<0>(leqcons_[row_id]) == rank_to_add) {
          next_eq_cost_vec_(old_rows+added_row) = std::get<1>(leqcons_[row_id]).getConstant();
          for (int var_id=0; var_id<(int)std::get<1>(leqcons_[row_id]).size(); var_id++)
            next_eq_cost_mat_(old_rows+added_row, std::get<1>(leqcons_[row_id]).getVar(var_id).get(solver::SolverIntParam_ColNum)) = std::get<1>(leqcons_[row_id]).getCoeff(var_id);
          added_row++;
        }
      assert(n_add_rows == added_row && "Incorrect number of appended linear equalities");
    }

    /*! appending linear inequalities */
    // count how many rows we add in this rank
    n_add_rows = 0;
    for (size_t row_id=0; row_id<lineqcons_.size(); row_id++)
      if (std::get<0>(lineqcons_[row_id]) == rank_to_add) { n_add_rows++; }

    if (n_add_rows > 0) {
      // extend the problem matrix by the amount of rows we need
      int old_rows = next_ineq_cost_mat_.rows();
      rt_solver::RtVectorUtils::conservativeResize(next_ineq_cost_vec_, old_rows+n_add_rows);
      rt_solver::RtMatrixUtils::conservativeResize(next_ineq_cost_mat_, old_rows+n_add_rows, next_ineq_cost_mat_.cols());
      next_ineq_cost_mat_.bottomRows(n_add_rows).setZero();
      next_ineq_cost_vec_.bottomRows(n_add_rows).setZero();

      // add the rows from mat that correspond with the requested rank
      int added_row = 0;
      for (size_t row_id=0; row_id<lineqcons_.size(); row_id++)
        if (std::get<0>(lineqcons_[row_id]) == rank_to_add) {
          next_ineq_cost_vec_(old_rows+added_row) = std::get<1>(lineqcons_[row_id]).getConstant();
          for (int var_id=0; var_id<(int)std::get<1>(lineqcons_[row_id]).size(); var_id++)
            next_ineq_cost_mat_(old_rows+added_row, std::get<1>(lineqcons_[row_id]).getVar(var_id).get(solver::SolverIntParam_ColNum)) = std::get<1>(lineqcons_[row_id]).getCoeff(var_id);
          added_row++;
        }
      assert(n_add_rows == added_row && "Incorrect number of appended linear inequalities");
    }
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::checkSlacknessOfHierarchy(const int rank)
  {
    for (size_t id=0; id<hierarchies_without_slacks_.size(); id ++)
      if (hierarchies_without_slacks_[id] == rank)
        hqp_solver_.applyIneqSlacks() = false;
    hqp_solver_.applyIneqSlacks() = true;
  }

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  void RtModel<Max_Ineq_Rows, Max_Eq_Rows, Num_OptVars>::print()
  {
    for (int leq_id=0; leq_id<leqcons_.size(); leq_id++) {
      std::cout << " rank " << std::get<0>(leqcons_[leq_id])
    		    << " cons " << std::get<1>(leqcons_[leq_id]) << std::endl;
    }

    for (int lineq_id=0; lineq_id<lineqcons_.size(); lineq_id++) {
      std::cout << " rank " << std::get<0>(lineqcons_[lineq_id])
    		    << " cons " << std::get<1>(lineqcons_[lineq_id]) << std::endl;
    }
  }

}
