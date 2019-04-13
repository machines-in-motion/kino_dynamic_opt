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

#include <solver/optimizer/BnBSolver.hpp>

namespace solver {

  //! Function to create branches for the new variable to explore
  void BnBSolver::createBranches(int node_id)
  {
    int partition_id = nodes_[node_id].partition_id_;

    nodes_[iteration_].lower_bound_ = nodes_[node_id].lower_bound_;
    nodes_[iteration_].upper_bound_ = nodes_[node_id].upper_bound_;
    nodes_[iteration_].status_ = Status::NotSolved;

    binvars_mat_id_.col(iteration_) = binvars_mat_id_.col(node_id);
    binvars_mat_id_(partition_id, iteration_) = static_cast<int>(Decision::One);
    binvars_mat_id_(partition_id, node_id) = static_cast<int>(Decision::Zero);
    nodes_[node_id].status_ = Status::NotSolved;
  }

  //! Function to select the node to be explored
  int BnBSolver::selectNodeToExplore()
  {
    int next_node = -1;
    double lower_bound = SolverSetting::inf;
    for (int node_id=0; node_id<=iteration_; node_id++) {
      if (nodes_[node_id].status_ == Status::SolvedBranchable &&
        nodes_[node_id].lower_bound_ < lower_bound) {
        next_node = node_id;
        lower_bound = nodes_[node_id].lower_bound_;
      }
    }
    return next_node;
  }

  //! This function computes the lower bound out of the nodes explored so far
  double BnBSolver::getProblemLowerBound()
  {
    double lower_bound = SolverSetting::inf;
    for (int node_id=0; node_id<=iteration_; node_id++)
      lower_bound = std::min(lower_bound, nodes_[node_id].lower_bound_);
    return lower_bound;
  }

  //! Function to select the variable to explore to refine the space search
  void BnBSolver::selectPartitionVariable(int& partition_id, double& partition_val)
  {
    double threshold = 1.0;
    for (int var_id=0; var_id<nbin_vars_; var_id++) {
      if (std::abs(this->getSolver().optimalVector().x()[binvars_ids_[var_id]]-0.5) < threshold){
        partition_id = var_id;
        partition_val = this->getSolver().optimalVector().x()[binvars_ids_[var_id]];
        threshold = std::abs(partition_val-0.5);
      }
    }
  }

  //! function to update problem data from node to explore
  void BnBSolver::updateProblemData(const Eigen::Ref<const Eigen::VectorXi>& node_id)
  {
    for (int var_id=0; var_id<nbin_vars_; var_id++){
      switch (node_id[var_id])
      {
        case static_cast<int>(Decision::One): {
          this->getSolver().updateEquilH(2*var_id,  -1.0);
          this->getSolver().updateEquilH(2*var_id+1, 1.0);
          break;
        }
        case static_cast<int>(Decision::Zero): {
          this->getSolver().updateEquilH(2*var_id,   0.0);
          this->getSolver().updateEquilH(2*var_id+1, 0.0);
          break;
        }
        case static_cast<int>(Decision::Undefined): {
          this->getSolver().updateEquilH(2*var_id,   0.0);
          this->getSolver().updateEquilH(2*var_id+1, 1.0);
          break;
        }
        default: { throw std::runtime_error("Incorrect value for BnB decision"); }
      }
    }
  }

  //! Stores the solution of the node with best statistics so far
  void BnBSolver::storeSolution()
  {
    this->getInfo() = this->getSolver().getInfo();
    opt_ = this->getSolver().optimalVector();
  }

  //! Loads back the solution of the node with best statistics
  void BnBSolver::loadSolution()
  {
    this->getSolver().getInfo() = this->getInfo();
    this->getSolver().optimalVector() = opt_;
  }

  void BnBSolver::updateNodeBounds(int node_id)
  {
    bool is_viable_sol = false;
    this->updateProblemData(binvars_mat_id_.col(node_id));
    opt_status_ = this->getSolver().optimize();

    if (opt_status_ == ExitCode::Optimal || opt_status_ == ExitCode::OptimalInacc ||
        opt_status_ == ExitCode::DualInf || opt_status_ == ExitCode::DualInfInacc ||
        opt_status_ == ExitCode::PrimalInf || opt_status_ == ExitCode::PrimalInfInacc ||
        opt_status_ == ExitCode::ReachMaxIters)
    {
      nodes_[node_id].lower_bound_ = this->getSolver().getStorage().c().dot(this->getSolver().optimalVector().x());

      bool is_integer_sol = true;
      for (int var_id=0; var_id<nbin_vars_; var_id++) {
        binvars_vec_id_[var_id] = std::round( this->getSolver().optimalVector().x()[binvars_ids_[var_id]] );
        is_integer_sol &= std::abs(this->getSolver().optimalVector().x()[binvars_ids_[var_id]] - binvars_vec_id_[var_id]) < this->getSolver().getSetting().get(SolverDoubleParam_BnBIntegerTol);
      }

      if (is_integer_sol) {
        nodes_[node_id].status_ = Status::SolvedNonBranchable;
        nodes_[node_id].upper_bound_ = this->getSolver().getStorage().c().dot(this->getSolver().optimalVector().x());
      } else {
        this->selectPartitionVariable(nodes_[node_id].partition_id_, nodes_[node_id].partition_val_);
        nodes_[node_id].status_ = Status::SolvedBranchable;

        this->updateProblemData(binvars_vec_id_);
        opt_status_ = this->getSolver().optimize();

        if (opt_status_ == ExitCode::Optimal || opt_status_ == ExitCode::OptimalInacc){
          nodes_[node_id].upper_bound_ = this->getSolver().getStorage().c().dot(this->getSolver().optimalVector().x());
          is_viable_sol = true;
        }
      }

      if (nodes_[node_id].upper_bound_ < prob_upper_bound_){
        storeSolution();
        prob_upper_bound_ = nodes_[node_id].upper_bound_;
      }

      if (is_viable_sol) { nodes_[node_id].upper_bound_ = SolverSetting::inf; }
    }
    else
    {
      nodes_[node_id].lower_bound_ = SolverSetting::inf;
      nodes_[node_id].upper_bound_ = SolverSetting::inf;
      nodes_[node_id].status_ = Status::SolvedNonBranchable;
    }
  }

  //! Convergence check in BnB routine search
  int BnBSolver::optimilityCheck(int node_id)
  {
    return (prob_upper_bound_ - prob_lower_bound_) > this->getSolver().getSetting().get(SolverDoubleParam_BnBAbsSubOptGap) &&
            std::abs(prob_upper_bound_ / prob_lower_bound_ - 1.0) > this->getSolver().getSetting().get(SolverDoubleParam_BnBRelSubOptGap) &&
            node_id >= 0 && iteration_ < (this->getSolver().getSetting().get(SolverIntParam_BnBMaxIters)-1);
  }

  //! Generate an exit condition for the problem, either full or reduced precision
  ExitCode BnBSolver::exitcode()
  {
    if ( iteration_ < this->getSolver().getSetting().get(SolverIntParam_BnBMaxIters)-1 ) {
      if ( std::isinf(prob_upper_bound_) ) {
        if ( prob_upper_bound_ >= 0) { return ExitCode::PrimalInf; }
        else { return ExitCode::DualInf; }
      } else return ExitCode::Optimal;
    } else {
      if ( std::isinf(prob_upper_bound_) ) {
        if ( prob_upper_bound_ >= 0) { return ExitCode::PrimalInfInacc; }
        else { return ExitCode::DualInfInacc; }
      } else return ExitCode::OptimalInacc;
    }
  }

  //! Initialize the root node of BnB solver
  void BnBSolver::initializeRootNode()
  {
    nodes_[0].status_ = Status::NotSolved;
    nodes_[0].lower_bound_ = -SolverSetting::inf;
    nodes_[0].upper_bound_ =  SolverSetting::inf;
    prob_lower_bound_ = -SolverSetting::inf;
    prob_upper_bound_ =  SolverSetting::inf;

    binvars_vec_id_.setConstant(static_cast<int>(Decision::Undefined));
    binvars_mat_id_.setConstant(static_cast<int>(Decision::Undefined));
  }

  // Stanford.edu/class/ee364b/lectures/bb_slides.pdf
  ExitCode BnBSolver::optimize()
  {
    iteration_ = 0, node_id_ = 0;
    initializeRootNode();
    updateNodeBounds(node_id_);
    prob_lower_bound_ = nodes_[node_id_].lower_bound_;
    prob_upper_bound_ = nodes_[node_id_].upper_bound_;

    while ( optimilityCheck(node_id_) )
    {
      ++iteration_;

      //! Create branches for the problem
      createBranches(node_id_);

      //! Updating problem bounds
      updateNodeBounds(node_id_);
      updateNodeBounds(iteration_);

      //! Decision on exploration node
      prob_lower_bound_ = getProblemLowerBound();
      node_id_ = selectNodeToExplore();
    }
    loadSolution();

    return exitcode();
  }

  void BnBSolver::initialize(InteriorPointSolver& interior_point_solver,
                             const std::vector< std::shared_ptr<Var> >& binary_variables)
  {
    interior_point_solver_ = &interior_point_solver;

    nbin_vars_ = binary_variables.size();
    opt_.initialize(this->getSolver().getCone());
    prob_upper_bound_ = SolverSetting::inf;
    nodes_.resize(this->getSolver().getSetting().get(SolverIntParam_BnBMaxIters));
    binvars_mat_id_.resize(nbin_vars_, this->getSolver().getSetting().get(SolverIntParam_BnBMaxIters));
    binvars_vec_id_.resize(nbin_vars_);

    binvars_ids_.resize(nbin_vars_);
    for (int var_id=0; var_id<nbin_vars_; var_id++)
      binvars_ids_(var_id) = binary_variables[var_id]->get(SolverIntParam_ColNum);
  }
}
