#pragma once

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include <solver/interface/Solver.hpp>
//#include <solver/optimizer/LbfgsSolver.hpp>
#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/setting/PlannerSetting.hpp>
//#include <momentumopt/cntopt/ContactPlanFromFile.hpp>
#include <momentumopt/kinopt/KinematicsInterface.hpp>

namespace momentumopt {

  /*! main class of the kinematic optimization problem  */
  class KinematicsOptimizer
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  /*! default class constructor and destructor */
	  KinematicsOptimizer(){}
      ~KinematicsOptimizer(){}

      /**
       * function to initialize and configure the optimization
       * @param[in]  PlannerSetting                       setting of planner configuration variables
       * @param[in]  kin_interface                        implementation of robot specific functions needed by the optimizer
       */
      void initialize(PlannerSetting& planner_setting, KinematicsInterface* kin_interface);

      /**
       * function to parse equations and objective into optimization problem and attempt to find a solution
       * @param[in]  ini_state                            initial state of the robot
       * @param[in]  dyn_sequence                         dynamics sequence to be used as momentum tracking reference
       * @param[in]  is_not_first_kindyn_iteration        signals if it is the first kinematics-dynamics iteration
       */
      void optimize(const DynamicsState& ini_state, const DynamicsSequence& dyn_sequence, bool is_not_first_kindyn_iteration = false);

      /**
       * this function gives access to the optimized motion plan
       * @return     DynamicsSequence                     reference to kinematic sequence, which is a collection of kinematic states,
       *                                                  each of which contains information about all variables, all end-effectors
       *                                                  of the motion plan, joint positions, velocities and accelerations for one time step
       */
      KinematicsSequence& kinematicsSequence() { return kin_sequence_; }
      const KinematicsSequence& kinematicsSequence() const { return kin_sequence_; }

//      /**
//       * helper function to replay a motion
//       * @param[in]  PlannerSetting                       setting of planner configuration variables
//       * @param[in]  kin_interface                        implementation of robot specific functions needed by the optimizer
//       * @param[in]  load_file                            file where the kinematic parameters have been stored
//       */
//      void displayMotion(PlannerSetting& planner_setting, KinematicsInterface* kin_interface, const std::string& load_file);
//
//      /**
//       * function to load a kinematic solution from a file and fill a sequence with it
//       * @param[in]  load_file                             name of configuration file from where to load the solution
//       */
//      void loadSolution(const std::string& load_file);

    private:
      /*! Getter and setter methods for getting the planner variables  */
      inline PlannerSetting& getSetting() { return *planner_setting_; }
      inline const PlannerSetting& getSetting() const { return *planner_setting_; }

      /**
       * function to initialize optimization variables: type [continuous or binary],
       * guess value [if any], upper and lower bounds for the variable
       */
      void initializeKinematicVariables();

      /**
       * function to add each variable to the Model and assign a unique identifier
       * to it, to be used by the optimizer to construct the problem
       * @param[in]  opt_var                              helper optimization variable for model predictive control
       * @param[in]  model                                instance of solver interface to collect constraints, objective and solve problem
       * @param[in]  vars                                 vector of optimization variables
       */
      void addVariableToModel(const solver::OptimizationVariable& opt_var, solver::Model& model, std::vector<solver::Var>& vars);

      /**
       * functions that transfers optimal solution from model to helper class OptimizationVariable,
       * and from helper class OptimizationVariable to dynamics sequence to be accessed by the user
       * @param[in]  opt_var                               helper optimization variable for model predictive control
       */
      void saveSolution(solver::OptimizationVariable& opt_var);
//      void storeSolution();

      /**
       * function that implements iterative inverse kinematics to track momentum,
       * end-effector motions, by a robot with a given joint configuration
       * @param[in/out]  current_state                     initial state and final state at end of iterative inverse kinematics
       * @param[in]      desired_state                     goal configuration to be achieved by iterative inverse kinematics
       * @param[in]      is_not_initial_state              flag to turn on/off momentum tracking for moving the robot to initial state
       * @param[in]      is_not_first_kinematic_iteration  flag to turn on/off momentum tracking for moving the robot to initial state
       */
      void optimizePosture(KinematicsState& current_state, const DynamicsState& desired_state, bool is_not_initial_state = true, bool include_torque_limits = false);

//      /*! helper function to display a kinematic motion */
//      void displayMotion();

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

      /*! exit code of the optimization problem */
      solver::ExitCode exitcode_;

      /**
       * c++ vector containing all problem variables defined by the user. Does not include extra
       * variables required to write the problem in standard conic form.
       */
      std::vector<solver::Var> vars_;

      /**
       * configuration of the robot, including center of mass position, linear and angular momenta,
       * end-effectors configurations: activation, position, orientation, joint configuration: positions,
       * velocities and accelerations, for initial state, current state and desired state
       */
      KinematicsState* current_state_; //ini_state_;

      /**
       * kinematics sequence of kinematics states. This is the main interface between the user
       * and the planner. The user can find in this variable all the optimization results
       * coming from the kinematics optimizer
       */
      KinematicsSequence kin_sequence_;

      /*! implementation of robot specific functions needed by the optimizer */
      KinematicsInterface* kin_interface_;

//      /*! lbfgs solver for dealing with end-effector rotations */
//      solver::LbfgsSolver eef_rot_kin_optimizer_;

      /*! type for optimization variable: continuous 'C' or binary 'B' */
      char variable_type_;

      /*! helper integer variables for the optimization problem */
      int size_, num_vars_;

      /*! helper optimization variables for the optimization problem */
      std::array<solver::OptimizationVariable, Problem::n_endeffs_> eef_pos_, eef_vel_, eef_ang_vel_;
      solver::OptimizationVariable total_qd_, total_qdd_, jnt_q_, jnt_qd_, com_, comd_, lmom_, amom_, base_ang_vel_;

      /*! helper matrices and vectors for the optimization problem */
      solver::OptimizationVariable::OptVector solution_;
      solver::OptimizationVariable::OptMatrix mat_guess_, mat_lb_, mat_ub_;
//
//      /*! helper matrices and vectors for the optimization problem */
//      Eigen::VectorXd nonlinear_terms_;
//      std::array<Eigen::MatrixXd, Problem::n_endeffs_> endeffector_jacobian_;
//      Eigen::MatrixXd inertia_matrix_, centroidal_momentum_matrix_, base_jacobian_;
  };

}
