/**
 * @file TestProblem00.hpp
 * @author Carl Laird, Andreas Waechter
 * @license Eclipse Public License
 * @copyright 2004, 2006 International Business Machines and others. All Rights Reserved.
 * @date 2004
 * 
 * $Id: MyNLP.hpp 1861 2010-12-21 21:34:47Z andreasw $
 */

#pragma once

#include <cassert>
#include <IpTNLP.hpp>

using namespace Ipopt;

/*
 * min_x f(x) = -(x2-2)^2
 *  s.t.
 *       0 = x1^2 + x2 - 1
 *       -1 <= x1 <= 1
 *
 */

namespace nlp_test_problems
{

  class TestProblem00 : public TNLP
  {
    public:
      TestProblem00(){}
      virtual ~TestProblem00(){}

      /** Method to return some info about the nlp */
      virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                Index& nnz_h_lag, IndexStyleEnum& index_style)
      {
        n = 2;           // number of variables
        m = 1;           // number of constraints
        nnz_jac_g = 2;   // nonzeros in the jacobian
        nnz_h_lag = 2;   // nonzeros in the hessian of the lagrangian
        index_style = FORTRAN_STYLE; // fortran index style for row/col entries

        return true;
      }

      /** Method to return the bounds for my problem */
      virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                   Index m, Number* g_l, Number* g_u)
      {
        assert(n == 2);
        assert(m == 1);

        // upper and lower bounds on variables
        x_l[0] = -1.0;
        x_u[0] = 1.0;
        x_l[1] = -1.0e19;
        x_u[1] = +1.0e19;

        // upper and lower bounds on constraints
        g_l[0] = g_u[0] = 0.0;

        return true;
      }

      /** Method to return the starting point for the algorithm */
      virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                      bool init_z, Number* z_L, Number* z_U,
                                      Index m, bool init_lambda,
                                      Number* lambda)
      {
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);

        x[0] = 0.5;
        x[1] = 1.5;

        return true;
      }

      /** Method to return the objective value */
      virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
      {
        Number x2 = x[1];
        obj_value = -(x2 - 2.0) * (x2 - 2.0);
        return true;
      }

      /** Method to return the gradient of the objective */
      virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
      {
        Number x2 = x[1];
        grad_f[0] = 0.0;
        grad_f[1] = -2.0*(x2 - 2.0);

        return true;
      }

      /** Method to return the constraint residuals */
      virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
      {
        Number x1 = x[0];
        Number x2 = x[1];
        g[0] = -(x1*x1 + x2 - 1.0);

        return true;
      }

      /** Method to return:
       *   1) The structure of the jacobian (if "values" is NULL)
       *   2) The values of the jacobian (if "values" is not NULL)
       */
      virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                              Index m, Index nele_jac, Index* iRow, Index *jCol,
                              Number* values)
      {
        if (values == NULL) {
          // return the structure of the jacobian of the constraints
          iRow[0] = 1;
          jCol[0] = 1;

          iRow[1] = 1;
          jCol[1] = 2;
        } else {
          // return the values of the jacobian of the constraints
          Number x1 = x[0];

          values[0] = -2.0 * x1;
          values[1] = -1.0;
        }
        return true;
      }

      /** Method to return:
       *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
       *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
       */
      virtual bool eval_h(Index n, const Number* x, bool new_x,
                          Number obj_factor, Index m, const Number* lambda,
                          bool new_lambda, Index nele_hess, Index* iRow,
                          Index* jCol, Number* values)
      {
        if (values == NULL) {
          // return the structure. This is a symmetric matrix, fill the lower left triangle only.
          iRow[0] = 1;
          jCol[0] = 1;

          iRow[1] = 2;
          jCol[1] = 2;
        } else {
          // return the values
          values[0] = -2.0 * lambda[0];
          values[1] = -2.0 * obj_factor;
        }
        return true;
      }

      virtual void finalize_solution(SolverReturn status,
                                     Index n, const Number* x, const Number* z_L, const Number* z_U,
                                     Index m, const Number* g, const Number* lambda,
                                     Number obj_value,
				                    const IpoptData* ip_data,
				                    IpoptCalculatedQuantities* ip_cq)
      {
      }

    private:
      TestProblem00(const TestProblem00&);
      TestProblem00& operator=(const TestProblem00&);
  };

}
