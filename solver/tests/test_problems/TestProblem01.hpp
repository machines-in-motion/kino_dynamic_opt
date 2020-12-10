/**
 * @file TestProblem01.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <limits>
#include <iostream>
#include <Eigen/Dense>

#include <solver/interface/NlpDescription.hpp>

namespace nlp_test_problems
{

  class TestProblem01 : public solver::NlpDescription
  {
    public:
	  TestProblem01(){};
      virtual ~TestProblem01(){};

      // definition of problem size
      void getNlpParameters(int& n_vars, int& n_cons)
      {
        n_vars = 4;
        n_cons = 2;
      }

      // definition of problem box constraints
      void getNlpBounds(int n_vars, int /*n_cons*/, double* x_l, double* x_u, double* g_l, double* g_u)
      {
        // lower and upper bounds on variables
        for (int id=0; id<n_vars; id++)
        {
          x_l[id] = 1.0;
          x_u[id] = 5.0;
        }

    	    // lower and upper bounds on inequality constraints
    	    g_l[0] = 25;
    	    g_u[0] = std::numeric_limits<double>::infinity();

    	    // lower and upper bounds on equality constraints
    	    g_l[1] = g_u[1] = 40.0;
      }

      // definition of starting point
      void getStartingPoint(int /*n_vars*/, double* x)
      {
    	    x[0] = 1.0;
    	    x[1] = 5.0;
    	    x[2] = 5.0;
    	    x[3] = 1.0;
      }

      // definition of objective function
      double evaluateObjective(int /*n_vars*/, const double* x)
      {
    	    return x[0]*x[3]*( x[0]+x[1]+x[2] ) + x[2];
      }

      // definition of constraints function
      void evaluateConstraintsVector(int /*n_vars*/, int /*n_cons*/, const double* x, double* constraints)
      {
    	    constraints[0] = x[0]*x[1]*x[2]*x[3];
    	    constraints[1] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
      }
  };
}
