/**
 * @file TestProblem02.hpp
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

  class TestProblem02 : public solver::NlpDescription
  {
    public:
	  TestProblem02(){};
      virtual ~TestProblem02(){};

      // definition of problem size
      void getNlpParameters(int& n_vars, int& n_cons)
      {
        n_vars = 2;
        n_cons = 0;
      }

      // definition of problem box constraints
      void getNlpBounds(int n_vars, int /*n_cons*/, double* x_l, double* x_u, double* /*g_l*/, double* /*g_u*/)
      {
    	    for (int i=0; i<n_vars; i++)
        {
          x_l[i] = -2.;
          x_u[i] =  2.;
        }
      }

      // definition of starting point
      void getStartingPoint(int /*n_vars*/, double* x)
      {
        x[0] =  0.0;
        x[1] = -1.0;
      }

      // definition of objective function
      double evaluateObjective(int /*n_vars*/, const double* x)
      {
        return 4.*pow(x[0],2.) - 2.1*pow(x[0],4.)+ 1./3.*pow(x[0],6.) + x[0]*x[1] - 4.*pow(x[1],2.) + 4.*pow(x[1],4.);
      }

      // definition of constraints function
      void evaluateConstraintsVector(int /*n_vars*/, int /*n_cons*/, const double* /*x*/, double* /*constraints*/)
      {
      }
  };
}
