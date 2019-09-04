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

#include <solver/interface/ConicProblem.hpp>
#include <solver/optimizer/NcvxBnBSolver.hpp>

namespace solver {

  class Model
  {
    public:
      Model(){}
      ~Model(){}

      void clean() { conic_problem_.clean(); }
      Var addVar(const VarType& type, double lb, double ub, double guess=0.0) { return conic_problem_.addVar(type, lb, ub, guess); }
      void addLinConstr(const LinExpr& lhs, const std::string sense, const LinExpr& rhs) { conic_problem_.addLinConstr(lhs, sense, rhs); }
      void addSocConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& lexpr) { conic_problem_.addSocConstr(qexpr, sense, lexpr); }
      void addQuaConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& expr, const QuadConstrApprox& qapprox = QuadConstrApprox::None ) { conic_problem_.addQuaConstr(qexpr, sense, expr, qapprox); }
      void configSetting(const std::string cfg_file, const std::string stg_vars_yaml = "solver_variables") { conic_problem_.configSetting(cfg_file, stg_vars_yaml); }
      void setObjective(const DCPQuadExpr& qexpr, const LinExpr& expr) { conic_problem_.setObjective(qexpr, expr); }
      ExitCode optimize();

      ConicProblem& getProblem() { return conic_problem_; }
      const ConicProblem& getProblem() const { return conic_problem_; }
      SolverSetting& getSetting() { return conic_problem_.getSetting(); }
      const SolverSetting& getSetting() const { return conic_problem_.getSetting(); }

    private:
      ConicProblem conic_problem_;
      NcvxBnBSolver ncvx_bnb_solver_;
  };
}
