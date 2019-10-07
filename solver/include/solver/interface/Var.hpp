/**
 * @file Var.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <memory>
#include <solver/interface/SolverParams.hpp>

namespace rt_solver {

  template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
  class RtModel;

}

namespace solver {

  enum class VarType {Binary, Continuous};

  struct VarStorage
  {
    int col_no_;
    VarType type_;
    double lb_, ub_, value_, guess_;
  };

  /**
   * Helper class to define an optimization variable, used in the
   * construction of linear and quadratic expressions.
   */
  class Var
  {
    public:
      Var() : var_storage_(nullptr) {};

      int get(SolverIntParam param) const;
      double get(SolverDoubleParam param) const;
      void set(SolverIntParam param, int value);
      void set(SolverDoubleParam param, double value);

      friend class LinExpr;
      friend class ConicProblem;

      template<int Max_Ineq_Rows, int Max_Eq_Rows, int Num_OptVars>
      friend class rt_solver::RtModel;

    private:
      Var(int col_no, const VarType& type, double lb, double ub, double guess=0.0);

    private:
	  std::shared_ptr<VarStorage> var_storage_;
  };
}
