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

#include <solver/interface/Exprs.hpp>
#include <solver/optimizer/IPSolver.hpp>
#include <solver/optimizer/BnBSolver.hpp>

namespace solver {

  class Model;
  class NcvxBnBSolver;

  /**
   * Main class to construct a second-order cone optimization problem.
   * It provides functionality for defining an objective function, linear
   * equality and inequality constraints, second-order cone constraints
   * and heuristics for solving problems with non-convex quadratic constraints.
   */
  class ConicProblem
  {
    public:
      ConicProblem() { this->clean(); }
      ~ConicProblem(){}

      void clean();
      SolverSetting& getSetting() { return stgs_; }
      const SolverSetting& getSetting() const { return stgs_; }
      Var addVar(const VarType& type, double lb, double ub, double guess=0.0);
      void addLinConstr(const LinExpr& lhs, const std::string sense, const LinExpr& rhs);
      void addSocConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& lexpr);
      void addQuaConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& expr, const QuadConstrApprox& qapprox = QuadConstrApprox::None );
      void configSetting(const std::string cfg_file, const std::string stg_vars_yaml = "solver_variables");
      void setObjective(const DCPQuadExpr& qexpr, const LinExpr& expr);
      ExitCode optimize();

      const int numTrustRegions() const { return numTrustRegions_; }
      const int numBinaryVariables() const { return bin_vars_.size(); }
      const int numSoftConstraints() const { return numSoftConstraints_; }

    private:
      friend class Model;
      friend class NcvxBnBSolver;

      inline Cone& getCone() { return cone_; }
      inline SolverStorage& getStorage() { return stg_; }
      InteriorPointSolver& getSolver() { return ip_solver_; }

      ExitCode solveProblem();
      void buildProblem(int iter_id, bool warm_start = false);

      // getter and setter methods
      Eigen::VectorXd& binaryLowerBounds() { return bin_vars_lower_bound_; }
      Eigen::VectorXd& binaryUpperBounds() { return bin_vars_upper_bound_; }
      std::vector<std::shared_ptr<Var>>& problemVariables() { return vars_; }
      std::vector<std::shared_ptr<Var>>& problemBinaryVariables() { return bin_vars_; }

      const Eigen::VectorXd& binaryLowerBounds() const { return bin_vars_lower_bound_; }
      const Eigen::VectorXd& binaryUpperBounds() const { return bin_vars_upper_bound_; }
      const std::vector<std::shared_ptr<Var>>& problemVariables() const { return vars_; }
      const std::vector<std::shared_ptr<Var>>& problemBinaryVariables() const { return bin_vars_; }

    private:
      Cone cone_;
      SolverStorage stg_;
      ExitCode exit_code_;
      SolverSetting stgs_;
      BnBSolver bnb_solver_;
      InteriorPointSolver ip_solver_;

      DCPQuadExpr objective_;
      int numTrustRegions_, numSoftConstraints_;
      std::vector<LinExpr> leqcons_, lineqcons_;
      std::vector<DCPQuadExpr> qineqcons_, soccons_;
      std::vector<std::shared_ptr<Var> > vars_, bin_vars_;
      Eigen::VectorXd bin_vars_lower_bound_, bin_vars_upper_bound_;
  };
}
