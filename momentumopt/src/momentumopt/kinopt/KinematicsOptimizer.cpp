/**
 * @file KinematicsOptimizer.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <iomanip>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <momentumopt/kinopt/KinematicsOptimizer.hpp>
#include <momentumopt/utilities/OrientationUtils.hpp>

using namespace solver;

namespace momentumopt {

  KinematicsOptimizer::~KinematicsOptimizer()
  {
    delete ini_state_;
    delete current_state_;
  }

  void KinematicsOptimizer::initialize(PlannerSetting& planner_setting, KinematicsInterface* kin_interface)
  {
    kin_interface_ = kin_interface;
    planner_setting_ = &planner_setting;
    kin_interface_->internalInitialization(planner_setting);

    ini_state_ = new KinematicsState(this->getSetting().get(PlannerIntParam_NumDofs));
    current_state_ = new KinematicsState(this->getSetting().get(PlannerIntParam_NumDofs));

    kinematicsSequence().resize(this->getSetting().get(PlannerIntParam_NumTimesteps), this->getSetting().get(PlannerIntParam_NumDofs));
    if (!this->getSetting().get(PlannerBoolParam_UseDefaultSolverSetting)) { model_.configSetting(this->getSetting().get(PlannerStringParam_ConfigFile)); }
    else                                                                   { model_.configSetting(this->getSetting().get(PlannerStringParam_DefaultSolverSettingFile)); }
  }

  void KinematicsOptimizer::optimize(const DynamicsState& ini_state, ContactPlanInterface* contact_plan, DynamicsSequence& dyn_sequence, bool is_first_kinopt)
  {
    contact_plan_ = contact_plan;
    contact_plan_->updateEndeffectorTrajectories(ini_state, dyn_sequence);

    *current_state_ = KinematicsState(this->getSetting().get(PlannerIntParam_NumDofs));
    current_state_->robotPosture().jointPositions() = this->getSetting().get(PlannerVectorParam_KinematicDefaultJointPositions);

    // optimization of initial posture
    this->optimizePosture(*current_state_, ini_state, true);
    *ini_state_ = *current_state_;

    if (is_first_kinopt) {
      // optimize kinematic postures
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        this->optimizePosture(*current_state_, dyn_sequence.dynamicsState(time_id), false);
        this->kinematicsSequence().kinematicsState(time_id) = *current_state_;
      }
    }

    // time-horizon motion optimization
    this->optimizeTrajectory(ini_state, dyn_sequence);
    if (this->getSetting().get(PlannerBoolParam_DisplayMotion)) { this->displayMotion(dyn_sequence); }

    if (this->getSetting().get(PlannerBoolParam_StoreData)) { this->storeSolution(ini_state, dyn_sequence); }
  }

  void KinematicsOptimizer::initializeTrajectoryKinematicVariables()
  {
    num_vars_ = 0.;
    double inf_value = SolverSetting::inf;

    com_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    lmom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    amom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);

    // end-effector positions and velocities
    for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
      eef_pos_[eff_id].initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
      eef_vel_[eff_id].initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    }

    jnt_q_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    jnt_qd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    total_qd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
    total_qdd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
  }

  void KinematicsOptimizer::optimizeTrajectory(const DynamicsState& ini_state, DynamicsSequence& dyn_sequence)
  {
    this->initializeTrajectoryKinematicVariables();
    for (int iter_id=0; iter_id<this->getSetting().get(PlannerIntParam_MaxKinTrajectoryIterations); iter_id++)
    {
      try
      {
        // add variables to model
        vars_.clear();
        for (int var_id=0; var_id<num_vars_; var_id++)
          vars_.push_back(Var());

        model_.clean();
        addVariableToModel(com_, model_, vars_);
        addVariableToModel(lmom_, model_, vars_);
        addVariableToModel(amom_, model_, vars_);

        addVariableToModel(jnt_q_, model_, vars_);
        addVariableToModel(jnt_qd_, model_, vars_);
        addVariableToModel(total_qd_, model_, vars_);
        addVariableToModel(total_qdd_, model_, vars_);

        for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          addVariableToModel(eef_pos_[eff_id], model_, vars_);
          addVariableToModel(eef_vel_[eff_id], model_, vars_);
        }

        // orientations in 3D space
        std::vector<Eigen::Vector3d> log_orientation;
        Eigen::Vector3d ini_log_orientation = this->quaternionToRotationVector(ini_state_->robotPosture().baseOrientation());
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          log_orientation.push_back( this->quaternionToRotationVector(this->kinematicsSequence().kinematicsState(time_id).robotPosture().baseOrientation()) );

        /*
         * PROBLEM OBJECTIVE
         */

        DCPQuadExpr quadobj;
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        {
          for (int axis_id=0; axis_id<3; axis_id++) {
            // penalty on center of mass and momentum tracking
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingCenterOfMass)[axis_id], dyn_sequence.dynamicsState(time_id).centerOfMass()[axis_id] - vars_[com_.id(axis_id,time_id)]);
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingLinearMomentum)[axis_id], dyn_sequence.dynamicsState(time_id).linearMomentum()[axis_id] - vars_[lmom_.id(axis_id,time_id)]);
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingAngularMomentum)[axis_id], dyn_sequence.dynamicsState(time_id).angularMomentum()[axis_id] - vars_[amom_.id(axis_id,time_id)]);
            if (time_id==0) {
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate)[axis_id], (vars_[lmom_.id(axis_id,time_id)] - ini_state.linearMomentum()[axis_id])/dyn_sequence.dynamicsState(time_id).time() );
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate)[axis_id], (vars_[amom_.id(axis_id,time_id)] - ini_state.angularMomentum()[axis_id])/dyn_sequence.dynamicsState(time_id).time() );
            } else {
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate)[axis_id], (vars_[lmom_.id(axis_id,time_id)] - vars_[lmom_.id(axis_id,time_id-1)])/dyn_sequence.dynamicsState(time_id).time() );
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate)[axis_id], (vars_[amom_.id(axis_id,time_id)] - vars_[amom_.id(axis_id,time_id-1)])/dyn_sequence.dynamicsState(time_id).time() );
            }

            // penalty for tracking desired end-effector positions
            for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
              if (!dyn_sequence.dynamicsState(time_id).endeffectorActivation(eff_id)) {
                quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingNonActiveEndeffectorPosition)[axis_id], dyn_sequence.dynamicsState(time_id).endeffectorPosition(eff_id)[axis_id] - vars_[eef_pos_[eff_id].id(axis_id,time_id)]);
              } else {
                quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition)[axis_id], dyn_sequence.dynamicsState(time_id).endeffectorPosition(eff_id)[axis_id] - vars_[eef_pos_[eff_id].id(axis_id,time_id)]);
              }
            }
          }

          // penalty on joint default posture
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumActiveDofs); dof_id++)
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicDefaultJointPositions)[dof_id], this->getSetting().get(PlannerVectorParam_KinematicDefaultJointPositions)[dof_id] - vars_[jnt_q_.id(6+dof_id,time_id)]);

          // penalty on joint velocities and accelerations
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightJointVelocity)[dof_id], vars_[total_qd_.id(dof_id,time_id)]);
            quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightJointAcceleration)[dof_id], vars_[total_qdd_.id(dof_id,time_id)]);
          }
        }
        model_.setObjective(quadobj, 0.0);

        /*
         * PROBLEM CONSTRAINTS
         */
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        {
          KinematicsState current_state = kin_interface_->updateJacobiansAndState(this->kinematicsSequence().kinematicsState(time_id), this->getSetting().get(PlannerDoubleParam_KinIntegrationStep));

          for (int axis_id=0; axis_id<3; axis_id++) {
            // center of mass and momentum
            lin_cons_ = -vars_[com_.id(axis_id,time_id)] + current_state.centerOfMass()[axis_id];
            for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++)
              lin_cons_ += this->getSetting().get(PlannerDoubleParam_KinIntegrationStep)*kin_interface_->centerOfMassJacobian()(axis_id, this->getSetting().get(PlannerIntVectorParam_ExtendedActiveDofs)[dof_id])*vars_[jnt_qd_.id(dof_id,time_id)];
            model_.addLinConstr(lin_cons_, "=",  0.0);

            lin_cons_ = -vars_[lmom_.id(axis_id,time_id)];
            for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++)
              lin_cons_ += kin_interface_->centroidalMomentumMatrix()(axis_id, this->getSetting().get(PlannerIntVectorParam_ExtendedActiveDofs)[dof_id])*vars_[total_qd_.id(dof_id,time_id)];
            model_.addLinConstr(lin_cons_, "=",  0.0);

            lin_cons_ = -vars_[amom_.id(axis_id,time_id)];
            for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++)
              lin_cons_ += kin_interface_->centroidalMomentumMatrix()(3+axis_id, this->getSetting().get(PlannerIntVectorParam_ExtendedActiveDofs)[dof_id])*vars_[total_qd_.id(dof_id,time_id)];
            model_.addLinConstr(lin_cons_, "=",  0.0);

            // end-effector positions, velocities and angular velocities
            for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
              lin_cons_ = vars_[eef_pos_[eff_id].id(axis_id,time_id)] - current_state.endeffectorPosition(eff_id)[axis_id] - this->getSetting().get(PlannerDoubleParam_KinIntegrationStep)*vars_[eef_vel_[eff_id].id(axis_id,time_id)];
              model_.addLinConstr(lin_cons_, "=",  0.0);

              lin_cons_ = -vars_[eef_vel_[eff_id].id(axis_id,time_id)];
              for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++)
                lin_cons_ += kin_interface_->endeffectorJacobian(eff_id)(axis_id, this->getSetting().get(PlannerIntVectorParam_ExtendedActiveDofs)[dof_id])*vars_[jnt_qd_.id(dof_id,time_id)];
              model_.addLinConstr(lin_cons_, "=",  0.0);
            }
          }

          // endeffector above the ground
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
            model_.addLinConstr(vars_[eef_pos_[eff_id].id(2,time_id)], ">", this->getSetting().get(PlannerDoubleParam_FloorHeight));

          // robot posture
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
            double reference = 0.0;
            if (dof_id>=0 && dof_id<3)      { reference = current_state.robotPosture().basePosition()[dof_id]; }
            else if (dof_id>=3 && dof_id<6) { reference = log_orientation[time_id][dof_id-3]; }
            else                            { reference = current_state.robotPosture().jointPositions()[this->getSetting().get(PlannerIntVectorParam_ActiveDofs)[dof_id-6]]; }
            lin_cons_ = vars_[jnt_q_.id(dof_id,time_id)] - (reference + this->getSetting().get(PlannerDoubleParam_KinIntegrationStep)*vars_[jnt_qd_.id(dof_id,time_id)]);
            model_.addLinConstr(lin_cons_, "=",  0.0);
          }

          // joint velocities
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
            if (time_id==0) {
              double reference = 0.0;
              if (dof_id>=0 && dof_id<3)      { reference = ini_state_->robotPosture().basePosition()[dof_id]; }
              else if (dof_id>=3 && dof_id<6) { reference = ini_log_orientation[dof_id-3]; }
              else                            { reference = ini_state_->robotPosture().jointPositions()[this->getSetting().get(PlannerIntVectorParam_ActiveDofs)[dof_id-6]]; }
              lin_cons_ = vars_[total_qd_.id(dof_id,time_id)] - (vars_[jnt_q_.id(dof_id,time_id)] - reference)/dyn_sequence.dynamicsState(time_id).time();
            } else {
              lin_cons_ = vars_[total_qd_.id(dof_id,time_id)] - (vars_[jnt_q_.id(dof_id,time_id)] - vars_[jnt_q_.id(dof_id,time_id-1)])/dyn_sequence.dynamicsState(time_id).time();
            }
            model_.addLinConstr(lin_cons_, "=",  0.0);
          }

          // joint accelerations
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
            if (time_id==0) { lin_cons_ = vars_[total_qdd_.id(dof_id,time_id)] - (vars_[total_qd_.id(dof_id,time_id)])/dyn_sequence.dynamicsState(time_id).time(); }
            else            { lin_cons_ = vars_[total_qdd_.id(dof_id,time_id)] - (vars_[total_qd_.id(dof_id,time_id)] - vars_[total_qd_.id(dof_id,time_id-1)])/dyn_sequence.dynamicsState(time_id).time(); }
            model_.addLinConstr(lin_cons_, "=",  0.0);
          }

          // joint limits
          for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumActiveDofs); dof_id++) {
            model_.addLinConstr(vars_[jnt_q_.id(6+dof_id,time_id)], ">", this->getSetting().get(PlannerVectorParam_MinJointAngles)[this->getSetting().get(PlannerIntVectorParam_ActiveDofs)[dof_id]]);
            model_.addLinConstr(vars_[jnt_q_.id(6+dof_id,time_id)], "<", this->getSetting().get(PlannerVectorParam_MaxJointAngles)[this->getSetting().get(PlannerIntVectorParam_ActiveDofs)[dof_id]]);
          }
        }

        // Write problem in standard conic form and solve it
        exitcode_ = model_.optimize();

        solution_.resize(num_vars_, 1);  solution_.setZero();
        for (int var_id=0; var_id<num_vars_; var_id++)
          solution_(var_id) = vars_[var_id].get(SolverDoubleParam_X);

        saveSolution(com_);
        saveSolution(lmom_);
        saveSolution(amom_);

        saveSolution(jnt_q_);
        saveSolution(jnt_qd_);
        saveSolution(total_qd_);
        saveSolution(total_qdd_);

        for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          saveSolution(eef_pos_[eff_id]);
          saveSolution(eef_vel_[eff_id]);
        }

        // integrate current generalized velocities
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            this->kinematicsSequence().kinematicsState(time_id).robotVelocity().generalizedJointVelocities()[jnt_id] = vars_[jnt_qd_.id(jnt_id,time_id)].get(SolverDoubleParam_X);

          this->kinematicsSequence().kinematicsState(time_id) = kin_interface_->integratePosture(this->kinematicsSequence().kinematicsState(time_id), this->getSetting().get(PlannerDoubleParam_KinIntegrationStep));
          if (time_id==0) { this->kinematicsSequence().kinematicsState(time_id) = kin_interface_->differentiatePostures(*ini_state_, this->kinematicsSequence().kinematicsState(time_id), dyn_sequence.dynamicsState(time_id).time()); }
          else            { this->kinematicsSequence().kinematicsState(time_id) = kin_interface_->differentiatePostures(this->kinematicsSequence().kinematicsState(time_id-1), this->kinematicsSequence().kinematicsState(time_id), dyn_sequence.dynamicsState(time_id).time()); }
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            this->kinematicsSequence().kinematicsState(time_id).robotAcceleration().generalizedJointAccelerations()[jnt_id] = vars_[total_qdd_.id(jnt_id,time_id)].get(SolverDoubleParam_X);
        }
      }
      catch(...)
      {
        std::cout << "Exception during trajectory-generation optimization" << std::endl;
      }

    }

  }

  void KinematicsOptimizer::initializePostureKinematicVariables()
  {
    num_vars_ = 0.;
    double inf_value = SolverSetting::inf;

    lmom_.initialize('C', 3, 1, -inf_value, inf_value, num_vars_);
    amom_.initialize('C', 3, 1, -inf_value, inf_value, num_vars_);
    slack_vars_.initialize('C', kin_interface_->constraintsVector().size(), 1, -inf_value, inf_value, num_vars_);
    jnt_q_.initialize('C', this->getSetting().get(PlannerIntParam_NumActiveDofs), 1, -inf_value, inf_value, num_vars_);
    jnt_qd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), 1, -inf_value, inf_value, num_vars_);
    jnt_qdd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), 1, -inf_value, inf_value, num_vars_);
    total_qd_.initialize('C', this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs), 1, -inf_value, inf_value, num_vars_);
  }

  void KinematicsOptimizer::optimizePosture(KinematicsState& current_state, const DynamicsState& desired_state, bool is_initial_state)
  {
    KinematicsState start_state = current_state;
    double cur_error, prev_error = SolverSetting::inf;

    for (int iter_id=0; iter_id<this->getSetting().get(PlannerIntParam_MaxKinConvergenceIterations); iter_id++)
    {
      current_state = kin_interface_->updateJacobiansAndState(current_state, this->getSetting().get(PlannerDoubleParam_KinIntegrationStep));
      desired_com_velocity_ = (desired_state.centerOfMass() - current_state.centerOfMass()) / this->getSetting().get(PlannerDoubleParam_KinIntegrationStep);
      desired_lmom_velocity_ = (desired_state.linearMomentum() - current_state.linearMomentum()) / this->getSetting().get(PlannerDoubleParam_KinIntegrationStep);
      desired_amom_velocity_ = (desired_state.angularMomentum() - current_state.angularMomentum()) / this->getSetting().get(PlannerDoubleParam_KinIntegrationStep);
      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
    	  desired_eff_velocity_[eff_id] = (desired_state.endeffectorPosition(eff_id) - current_state.endeffectorPosition(eff_id)) / this->getSetting().get(PlannerDoubleParam_KinIntegrationStep);

      cur_error = desired_com_velocity_.norm() + desired_amom_velocity_.norm() + desired_lmom_velocity_.norm();
      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
        cur_error += desired_eff_velocity_[eff_id].norm();

      if (cur_error<this->getSetting().get(PlannerDoubleParam_KinConvergenceTolerance) ||
          std::abs(prev_error-cur_error)<this->getSetting().get(PlannerDoubleParam_KinConvergenceTolerance))
      { break; }
      prev_error = cur_error;

      int num_constraints = kin_interface_->constraintsVector().size();
      this->initializePostureKinematicVariables();

      try
      {
        // add variables to model
        vars_.clear();
        for (int var_id=0; var_id<num_vars_; var_id++)
          vars_.push_back(Var());

        model_.clean();
        addVariableToModel(lmom_, model_, vars_);
        addVariableToModel(amom_, model_, vars_);
        addVariableToModel(jnt_q_, model_, vars_);
        addVariableToModel(jnt_qd_, model_, vars_);
        addVariableToModel(jnt_qdd_, model_, vars_);
        addVariableToModel(total_qd_, model_, vars_);
        addVariableToModel(slack_vars_, model_, vars_);

        /*
         *  PROBLEM OBJECTIVE
         */
        DCPQuadExpr quadobj;

        for (int axis_id=0; axis_id<3; axis_id++)
        {
          // center of mass tracking
          lin_cons_ = -desired_com_velocity_[axis_id];
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            lin_cons_ += kin_interface_->centerOfMassJacobian()(axis_id,jnt_id)*vars_[jnt_qd_.id(jnt_id,0)];
          quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingCenterOfMass)[axis_id], lin_cons_);

          // linear momentum tracking
          lin_cons_ = -desired_lmom_velocity_[axis_id];
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            lin_cons_ += kin_interface_->centroidalMomentumMatrix()(axis_id,jnt_id)*vars_[jnt_qdd_.id(jnt_id,0)]
                       + kin_interface_->centroidalMomentumMatrixVariation()(axis_id,jnt_id)*vars_[total_qd_.id(jnt_id,0)];
          quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingLinearMomentum)[axis_id], lin_cons_);

          // angular momentum tracking
          lin_cons_ = -desired_amom_velocity_[axis_id];
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            lin_cons_ += kin_interface_->centroidalMomentumMatrix()(3+axis_id,jnt_id)*vars_[jnt_qdd_.id(jnt_id,0)]
                       + kin_interface_->centroidalMomentumMatrixVariation()(3+axis_id,jnt_id)*vars_[total_qd_.id(jnt_id,0)];
          quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingAngularMomentum)[axis_id], lin_cons_);

          // end-effector motion tracking
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            lin_cons_ = -desired_eff_velocity_[eff_id][axis_id];
            for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
              lin_cons_ += kin_interface_->endeffectorJacobians()[eff_id](axis_id,jnt_id)*vars_[jnt_qd_.id(jnt_id,0)];
            if (desired_state.endeffectorActivation(eff_id)) {
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition)[axis_id], lin_cons_);
            } else {
              quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicTrackingNonActiveEndeffectorPosition)[axis_id], lin_cons_);
            }
          }
        }

        // penalty over controls
        for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
          quadobj.addQuaTerm(this->getSetting().get(PlannerDoubleParam_LambdaRegularization), vars_[jnt_qd_.id(dof_id,0)]);
          quadobj.addQuaTerm(this->getSetting().get(PlannerDoubleParam_LambdaRegularization), vars_[jnt_qdd_.id(dof_id,0)]);
        }

        // penalty over slack variables
        for (int con_id=0; con_id<num_constraints; con_id++)
          quadobj.addQuaTerm(this->getSetting().get(PlannerDoubleParam_KinSlacksPenalty), vars_[slack_vars_.id(con_id,0)]);

        // penalty over default posture
        for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumActiveDofs); jnt_id++) {
          quadobj.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightKinematicDefaultJointPositions)[jnt_id], vars_[jnt_q_.id(jnt_id,0)] - this->getSetting().get(PlannerVectorParam_KinematicDefaultJointPositions)[jnt_id]);
        }

        model_.setObjective(quadobj, 0.0);

        /*
         *  PROBLEM CONSTRAINTS
         */

        // integration of velocities using the accelerations
        for (int dof_id=0; dof_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); dof_id++) {
          lin_cons_ = -vars_[total_qd_.id(dof_id,0)] + current_state.robotVelocity().generalizedJointVelocities()[dof_id] + vars_[jnt_qdd_.id(dof_id,0)]*this->getSetting().get(PlannerDoubleParam_KinIntegrationStep);
          //model_.addLinConstr(lin_cons_, "=", 0.0);
        }

        // bodies non-penetration constraints
        for (int con_id=0; con_id<num_constraints; con_id++) {
          lin_cons_ = -kin_interface_->constraintsVector()[con_id];
          for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
            lin_cons_ += kin_interface_->constraintsMatrix().row(con_id)[jnt_id]*vars_[jnt_qd_.id(jnt_id,0)];
          model_.addLinConstr(vars_[slack_vars_.id(con_id,0)], ">", 0.0);
          model_.addLinConstr(lin_cons_, "<", vars_[slack_vars_.id(con_id,0)]);
        }

        // integrating joint positions and adding joint limits
        for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumActiveDofs); jnt_id++) {
          lin_cons_ = -vars_[jnt_q_.id(jnt_id,0)] + current_state.robotPosture().jointPositions()[jnt_id] + this->getSetting().get(PlannerDoubleParam_KinIntegrationStep)*vars_[jnt_qd_.id(6+jnt_id,0)];
          model_.addLinConstr(lin_cons_, "=", 0.0);
          model_.addLinConstr(vars_[jnt_q_.id(jnt_id,0)], ">", this->getSetting().get(PlannerVectorParam_MinJointAngles)[jnt_id]);
          model_.addLinConstr(vars_[jnt_q_.id(jnt_id,0)], "<", this->getSetting().get(PlannerVectorParam_MaxJointAngles)[jnt_id]);
        }

        // bodies non-penetration constraints// Write problem in standard conic form and solve it
        exitcode_ = model_.optimize();

        solution_.resize(num_vars_, 1);  solution_.setZero();
        for (int var_id=0; var_id<num_vars_; var_id++)
          solution_(var_id) = vars_[var_id].get(SolverDoubleParam_X);

        saveSolution(lmom_);
        saveSolution(amom_);
        saveSolution(jnt_q_);
        saveSolution(jnt_qd_);
        saveSolution(jnt_qdd_);
        saveSolution(total_qd_);
        saveSolution(slack_vars_);

        // storing solution
        for (int jnt_id=0; jnt_id<this->getSetting().get(PlannerIntParam_NumExtendedActiveDofs); jnt_id++)
          current_state.robotVelocity().generalizedJointVelocities()[jnt_id] = vars_[jnt_qd_.id(jnt_id,0)].get(SolverDoubleParam_X);

        // integrate current step velocities and update total velocities
        current_state = kin_interface_->integratePosture(current_state, this->getSetting().get(PlannerDoubleParam_KinIntegrationStep));
        current_state = kin_interface_->differentiatePostures(start_state, current_state, desired_state.time());
      }
      catch(...)
      {
        std::cout << "Exception during posture-generation optimization" << std::endl;
      }
    }

    if (is_initial_state) {
      current_state.linearMomentum() = desired_state.linearMomentum();
      current_state.angularMomentum() = desired_state.angularMomentum();
      current_state.robotVelocity().generalizedJointVelocities().setZero();
      current_state.robotAcceleration().generalizedJointAccelerations().setZero();
    }
  }

  void KinematicsOptimizer::addVariableToModel(const OptimizationVariable& opt_var, Model& model, std::vector<Var>& vars)
  {
    opt_var.getValues(mat_lb_, mat_ub_, mat_guess_, size_, variable_type_);
    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
        switch (variable_type_) {
          case 'C': { vars[opt_var.id(row_id,col_id)] = model.addVar(VarType::Continuous, double(mat_lb_(row_id,col_id)), double(mat_ub_(row_id,col_id)), double(mat_guess_(row_id,col_id))); break; }
          case 'B': { vars[opt_var.id(row_id,col_id)] = model.addVar(VarType::Binary, int(mat_lb_(row_id,col_id)), int(mat_ub_(row_id,col_id)), double(mat_guess_(row_id,col_id))); break; }
          default: { throw std::runtime_error("At addVariableToModel, variable type not handled"); }
        }

    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
        vars[opt_var.id(row_id,col_id)].set(SolverDoubleParam_X, mat_guess_(row_id,col_id));
  }

  void KinematicsOptimizer::saveSolution(OptimizationVariable& opt_var)
  {
	mat_guess_.resize(opt_var.getNumRows(), opt_var.getNumCols());
	mat_guess_.setZero();
    for (int col_id=0; col_id<opt_var.getNumCols(); col_id++) {
      for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
    	    mat_guess_(row_id,col_id) = solution_(opt_var.id(row_id,col_id));
    }
    opt_var.setGuessValue(mat_guess_);
  }

  void KinematicsOptimizer::displayMotion(const DynamicsSequence& dyn_sequence)
  {
    int num_dofs = this->getSetting().get(PlannerIntParam_NumDofs);
    KinematicsState state0(num_dofs), state1(num_dofs), state2(num_dofs);
    for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
      if (time_id==0) {
        state0 = *ini_state_;
        state1 = this->kinematicsSequence().kinematicsState(time_id);
      } else {
        state0 = this->kinematicsSequence().kinematicsState(time_id-1);
        state1 = this->kinematicsSequence().kinematicsState(time_id);
      }
      for (int sample_id=0; sample_id<this->getSetting().get(PlannerIntParam_NumSubsamples); sample_id++) {
        double factor = double(sample_id) / double(this->getSetting().get(PlannerIntParam_NumSubsamples));
        state2.robotPosture().baseOrientation() = state0.robotPosture().baseOrientation().slerp(factor, state1.robotPosture().baseOrientation());
        state2.robotPosture().basePosition() = state0.robotPosture().basePosition() + factor*(state1.robotPosture().basePosition() - state0.robotPosture().basePosition());
        state2.robotPosture().jointPositions() = state0.robotPosture().jointPositions() + factor*(state1.robotPosture().jointPositions() - state0.robotPosture().jointPositions());
        kin_interface_->displayPosture(state2, dyn_sequence.dynamicsState(time_id).time()/this->getSetting().get(PlannerIntParam_NumSubsamples));
      }
    }
  }

  void KinematicsOptimizer::statesToDynamicsSequence(DynamicsSequence& dyn_sequence, Eigen::VectorXd com, Eigen::VectorXd lmom, Eigen::VectorXd amom) {
      auto num_vars = dyn_sequence.size();
      std::cout << num_vars << std::endl;
  }

  void KinematicsOptimizer::storeSolution(const DynamicsState& ini_state, DynamicsSequence& dyn_sequence)
  {
    try
    {
      YAML::Node qcqp_cfg;

      // config parameters
      qcqp_cfg["time_step"] = this->getSetting().get(PlannerDoubleParam_TimeStep);
      qcqp_cfg["time_horizon"] = this->getSetting().get(PlannerDoubleParam_TimeHorizon);
      qcqp_cfg["num_subsamples"] = this->getSetting().get(PlannerIntParam_NumSubsamples);

      // saving initial state
      qcqp_cfg["initial_robot_configuration"]["generalized_joint_positions"] = ini_state_->robotPosture().generalizedJointPositions();

      // saving dynamic sequence
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        qcqp_cfg["dyn_state_"+std::to_string(time_id)]["time"] = dyn_sequence.dynamicsState(time_id).time();
        qcqp_cfg["dyn_state_"+std::to_string(time_id)]["generalized_joint_positions"] = this->kinematicsSequence().kinematicsState(time_id).robotPosture().generalizedJointPositions();
        qcqp_cfg["dyn_state_"+std::to_string(time_id)]["generalized_joint_velocities"] = this->kinematicsSequence().kinematicsState(time_id).robotVelocity().generalizedJointVelocities();
        qcqp_cfg["dyn_state_"+std::to_string(time_id)]["generalized_joint_accelerations"] = this->kinematicsSequence().kinematicsState(time_id).robotAcceleration().generalizedJointAccelerations();
      }
      std::ofstream file_out(this->getSetting().get(PlannerStringParam_SaveKinematicsFile));
      file_out << qcqp_cfg;
    }
    catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }
  }

//  void KinematicsOptimizer::loadSolution(const std::string& load_file)
//  {
//    try
//	{
//    	  YAML::Node qcqp_cfg = YAML::LoadFile(load_file.c_str());
//
//    	  // config parameters
//    	  readParameter(qcqp_cfg, "time_step", this->getSetting().get(PlannerDoubleParam_TimeStep));
//    	  readParameter(qcqp_cfg, "time_horizon", this->getSetting().get(PlannerDoubleParam_TimeHorizon));
//    	  readParameter(qcqp_cfg, "num_subsamples", this->getSetting().get(PlannerIntParam_NumSubsamples));
//    	  this->getSetting().get(PlannerIntParam_NumTimesteps) = std::floor(this->getSetting().get(PlannerDoubleParam_TimeHorizon)/this->getSetting().get(PlannerDoubleParam_TimeStep));
//    	  kinematicsSequence().resize(this->getSetting().get(PlannerIntParam_NumTimesteps));
//
//	  // loading initial state
//    	  YAML::Node ini_params = qcqp_cfg["initial_robot_configuration"];
//    	  ini_state_.time() = this->getSetting().get(PlannerDoubleParam_TimeStep);
//	  readParameter(ini_params, "center_of_mass", ini_state_.centerOfMass());
//	  readParameter(ini_params, "joint_positions", ini_state_.jointPositions());
//	  Eigen::Vector4d v = readParameter<Eigen::Vector4d>(ini_params, "base_orientation");
//	  ini_state_.baseOrientation() = Eigen::Quaternion<double>(v[0], v[1], v[2], v[3]);
//
//	  // loading dynamic sequence
//	  for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
//		YAML::Node dyn_params = qcqp_cfg["dyn_state_"+std::to_string(time_id)];
//		readParameter(dyn_params, "time", kinematicsSequence().dynamicsState(time_id).time());
//		readParameter(dyn_params, "center_of_mass", kinematicsSequence().dynamicsState(time_id).centerOfMass());
//		readParameter(dyn_params, "joint_positions", kinematicsSequence().dynamicsState(time_id).jointPositions());
//		readParameter(dyn_params, "joint_velocities", kinematicsSequence().dynamicsState(time_id).jointVelocities());
//        readParameter(dyn_params, "joint_accelerations", kinematicsSequence().dynamicsState(time_id).jointAccelerations());
//		Eigen::Vector4d v = readParameter<Eigen::Vector4d>(ini_params, "base_orientation");
//		kinematicsSequence().dynamicsState(time_id).baseOrientation() = Eigen::Quaternion<double>(v[0], v[1], v[2], v[3]);
//	  }
//	}
//    catch (std::runtime_error& e)
//    {
//      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
//    }
//  }
//
//  void KinematicsOptimizer::displayMotion(PlannerSetting& planner_setting, KinematicsInterface* kin_interface, const std::string& load_file)
//  {
//	kin_interface_ = kin_interface;
//	planner_setting_ = &planner_setting;
//	this->loadSolution(load_file);
//	this->displayMotion();
//  }

  Eigen::Vector3d KinematicsOptimizer::quaternionToRotationVector(const Eigen::Quaternion<double>& q) const
  {
    return kin_interface_->logarithmicMap(Eigen::Vector4d(q.w(), q.x(),q.y(),q.z()));
  }

}
