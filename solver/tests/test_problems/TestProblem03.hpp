/**
 * @file TestProblem03.hpp
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

  class TestProblem03 : public solver::NlpDescription
  {
    public:
	  TestProblem03(){};
      virtual ~TestProblem03(){};

      // definition of problem size
      void getNlpParameters(int& n_vars, int& n_cons)
      {
        n_vars = 100;
        n_cons = 0;
      }

      // definition of problem box constraints
      void getNlpBounds(int n_vars, int n_cons, double* x_l, double* x_u, double* g_l, double* g_u)
      {
        for (int i=0; i<n_vars; i++)
        {
          x_l[i] = -2.;
          x_u[i] =  2.;
        }
      }

      // definition of starting point
      void getStartingPoint(int n_vars, double* x)
      {
      	for (int i=0; i<n_vars; i+=2)
      	{
      	  x[i]   = -1.2;
      	  x[i+1] =  1.0;
      	}
      }

      // definition of objective function
      double evaluateObjective(int n_vars, const double* x)
      {
        double fx = 0.0;
        for (int i=0; i<n_vars; i+=2) {
          double t1 = 1.0 - x[i];
          double t2 = 10.0 * (x[i+1] - x[i] * x[i]);
          fx += t1*t1 + t2*t2;
        }
        return fx;
      }

      // definition of constraints function
      void evaluateConstraintsVector(int n_vars, int n_cons, const double* x, double* constraints)
      {
      }
  };
}
