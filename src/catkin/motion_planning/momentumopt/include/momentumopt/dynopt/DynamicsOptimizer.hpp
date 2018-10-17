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

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include <solver/interface/Solver.hpp>
#include <momentumopt/utilities/Clock.hpp>
#include <momentumopt/setting/PlannerSetting.hpp>
#include <momentumopt/kinopt/KinematicsState.hpp>
#include <momentumopt/cntopt/ContactPlanInterface.hpp>

namespace momentumopt {

  /**
   * Helper class to define a linear approximation of a friction cone.
   */
  struct FrictionCone
  {
    typedef Eigen::Matrix< double, 4, 3> ConeMat;
    void getCone(double fcoeff, FrictionCone::ConeMat& cone_mat)
    {
      if (fcoeff <= 0.001) { fcoeff = 0.001; }
      Eigen::Vector3d fconevec = Eigen::Vector3d(0.5*sqrt(2.0), 0.5*sqrt(2.0), -fcoeff); fconevec.normalize();
      Eigen::Matrix3d fconemat = Eigen::Matrix3d::Identity();
      double angle = 2*M_PI/4;
      fconemat(0,0) = cos(angle);    fconemat(0,1) = -sin(angle);
      fconemat(1,0) = sin(angle);    fconemat(1,1) =  cos(angle);
      for (int i=0; i<4; i++) {
        cone_mat.row(i) = fconevec;
        fconevec = fconemat * fconevec;
      }
    }
  };

  /*! main class of the dynamics optimization problem  */
  class DynamicsOptimizer
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
      /*! default class constructor and destructor */
	  DynamicsOptimizer(){}
      ~DynamicsOptimizer(){}

      /**
       * function to initialize and configure the optimization
       * @param[in]  PlannerSetting               setting of planner configuration variables
       */
      void initialize(PlannerSetting& planner_setting);

      /**
       * function to parse equations and objective into optimization problem and attempt to find a solution
       * @param[in]  ini_state                    initial state of the robot
       * @param[in]  contact_plan                 container of contact sequence to be used for dynamics planning
       * @param[in]  kin_sequence                 kinematics sequence to be used as momentum tracking reference (can be zeros).
       * @param[in]  update_tracking_objective    changes weights from regulation to tracking of momentum in ref_sequence
       * @return     ExitCode                     flag that indicates the optimization result (for example: optimal, infeasible)
       */
      solver::ExitCode optimize(const DynamicsState& ini_state, ContactPlanInterface* contact_plan,
                                const KinematicsSequence& kin_sequence, bool update_tracking_objective = false);

      /**
       * this function gives access to the optimized motion plan
       * @return     DynamicsSequence             reference to dynamics sequence, which is a collection of dynamic states,
       *                                          each of which contains information about all variables, all end-effectors
       *                                          of the motion plan for one time step
       */
      DynamicsSequence& dynamicsSequence() { return dyn_sequence_; }
      const DynamicsSequence& dynamicsSequence() const { return dyn_sequence_; }

      /*! function to have access to time required to solve the optimization problem */
      const double& solveTime() const { return solve_time_; }

    private:
      /*! Getter and setter methods for getting the planner variables  */
      inline PlannerSetting& getSetting() { return *planner_setting_; }
      inline const PlannerSetting& getSetting() const { return *planner_setting_; }

      /*! helper functions for the optimization problem */
      const ContactType contactType(int time_id, int eff_id) const;
      const Eigen::Matrix3d contactRotation(int time_id, int eff_id) const;
      const double contactLocation(int time_id, int eff_id, int axis_id) const;

      /**
       * function to initialize optimization variables: type [continuous or binary],
       * guess value [if any], upper and lower bounds for the variable
       */
      void initializeOptimizationVariables();

      /**
       * function to add each variable to the Model and assign a unique identifier
       * to it, to be used by the optimizer to construct the problem
       * @param[in]  opt_var                      helper optimization variable for model predictive control
       * @param[in]  model                        instance of solver interface to collect constraints, objective and solve problem
       * @param[in]  vars                         vector of optimization variables
       */
      void addVariableToModel(const solver::OptimizationVariable& opt_var, solver::Model& model, std::vector<solver::Var>& vars);

      /**
       * functions to update tracking objective for momentum from penalty to tracking
       * and attempt to find a solution to the optimization problem
       * @param[in]  ref_sequence                 dynamics sequence to be used as momentum tracking reference (can be zeros).
       * @param[in]  is_first_time                flag to indicate if this is the first time the solution is being constructed
       */
      void internalOptimize(const KinematicsSequence& kin_sequence, bool is_first_time = false);
      void updateTrackingObjective();

      /**
       * functions that transfers optimal solution from model to helper class OptimizationVariable,
       * from helper class OptimizationVariable to dynamics sequence to be accessed by the user, and
       * from dynamics sequence to a file.
       * @param[in]  opt_var                      helper optimization variable for model predictive control
       * @param[in]  ref_sequence                 dynamics sequence to be used as momentum tracking reference (can be zeros).
       */
      void saveSolution(solver::OptimizationVariable& opt_var);
      void storeSolution();
      void saveToFile(const KinematicsSequence& kin_sequence);

    private:
      /*! Variables required for optimization */
      PlannerSetting* planner_setting_;

      /**
       * class that stores all information about the problem, interfaces between high level definition
       * of the problem and its corresponding mathematical construction to solve it with a solver instance
       */
      solver::Model model_;

      /*! helper variables to construct linear and quadratic expressions */
      solver::LinExpr lin_cons_;
      solver::DCPQuadExpr quad_objective_, quad_cons_;

      /*! exit code of the optimization problem */
      solver::ExitCode exitcode_;

      /**
       * c++ vector containing all problem variables defined by the user. Does not include extra
       * variables required to write the problem in standard conic form.
       */
      std::vector<solver::Var> vars_;

      /**
       * initial configuration of the robot, including center of mass position, linear and angular momenta,
       * end-effectors configurations: activation, position, orientation
       */
      DynamicsState ini_state_;

      /*! simple helper class to build a linear approximation of a friction cone */
      FrictionCone friction_cone_;
      FrictionCone::ConeMat cone_matrix_;

      /**
       * dynamics sequence of dynamic states. This is the main interface between the user
       * and the planner. The user can find in this variable all the optimization results
       */
      DynamicsSequence dyn_sequence_;

      /**
       * helper class to store information about the contact plan, such as surfaces,
       * position and orientation of a sequence of contacts
       */
      ContactPlanInterface* contact_plan_;

      /*! clock for timing purposes */
      Clock timer_;

      /*! type for optimization variable: continuous 'C' or binary 'B' and type of heuristic */
      char variable_type_;

      /*! helper boolean variables for the optimization problem */
      bool has_converged_;

      /*! helper integer variables for the optimization problem */
      int size_, num_vars_;

      /*! helper double variables for the optimization problem */
      double solve_time_, convergence_err_, last_convergence_err_;

      /*! helper vector variables for the optimization problem */
      Eigen::Vector3d com_pos_goal_;

      /*! helper optimization variables for the optimization problem */
      solver::OptimizationVariable dt_, com_, lmom_, amom_, lmomd_, amomd_;
      std::array<solver::OptimizationVariable, Problem::n_endeffs_> frc_world_, trq_local_, cop_local_, ub_var_, lb_var_;

      /*! helper matrices and vectors for the optimization problem */
      solver::OptimizationVariable::OptVector solution_;
      solver::OptimizationVariable::OptMatrix mat_lb_, mat_ub_, mat_guess_, com_guess_, lmom_guess_, amom_guess_;
  };

}
