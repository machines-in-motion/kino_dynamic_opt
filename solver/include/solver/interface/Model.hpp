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
