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

#include <iostream>
#include <Eigen/Dense>

namespace solver
{

  // Nonlinear problem description
  class NlpDescription
  {
    public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  NlpDescription(){};
      virtual ~NlpDescription(){};

      // definition of problem size
      virtual void getNlpParameters(int& n_vars, int& n_cons) = 0;

      // definition of problem box constraints
      virtual void getNlpBounds(int n_vars, int n_cons, double* x_l, double* x_u, double* g_l, double* g_u) = 0;

      // definition of starting point
      virtual void getStartingPoint(int n_vars, double* x) = 0;

      // definition of objective function
      virtual double evaluateObjective(int n_vars, const double* x) = 0;

      // definition of constraints function
      virtual void evaluateConstraintsVector(int n_vars, int n_cons, const double* x, double* constraints) = 0;

      // definition of how to process solution
      virtual void processSolution(int n_vars, double objective_value, const double* x)
      {
        Eigen::Map<const Eigen::VectorXd> eig_x_const(&x[0], n_vars);
        this->optimalVector() = eig_x_const;
        this->optimalValue()  = objective_value;
      }

      double& optimalValue() { return eig_obj_; }
      const double& optimalValue() const { return eig_obj_; }
      Eigen::VectorXd& optimalVector() { return eig_opt_x_; }
      const Eigen::VectorXd& optimalVector() const { return eig_opt_x_; }

    private:
      double eig_obj_;
      Eigen::VectorXd eig_opt_x_;
  };
}
