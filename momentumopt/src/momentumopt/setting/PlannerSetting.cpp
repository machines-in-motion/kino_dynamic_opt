/**
 * @file PlannerSetting.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <momentumopt/setting/PlannerSetting.hpp>

namespace momentumopt {

  // PlannerSetting class function implementations
  void PlannerSetting::initialize(const std::string& cfg_file, const std::string& planner_vars_yaml)
  {
    cfg_file_ = cfg_file;
    // save_dynamics_file_ = cfg_file.substr(0, cfg_file.size()-5) + "_dyn_results.yaml";
    save_kinematics_file_ = cfg_file.substr(0, cfg_file.size()-5) + "_kin_results.yaml";
    default_solver_setting_file_ = CFG_SRC_PATH + std::string("default_solver_setting.yaml");

    try
    {
      // Load the Paramter file and make sure the error is understandable
      YAML::Node planner_cfg;
      try { planner_cfg = YAML::LoadFile(cfg_file.c_str()); }
      catch (std::runtime_error& e) {
          throw std::runtime_error(
            "Error loading the yaml file " + cfg_file + " with error: " +
            e.what() );
      }
      // load the local node
      YAML::Node planner_vars;
      try { planner_vars = planner_cfg[planner_vars_yaml.c_str()]; }
      catch (std::runtime_error& e) {
          throw std::runtime_error(
            "Error getting the planner_vars_yaml [" + planner_vars_yaml +
            "] with error: " + e.what());
      }

	  // Kinematics parameters
	  YAML::ReadParameter(planner_vars, "load_kinematics", load_kinematics_);
      if (load_kinematics_) {
	    YAML::ReadParameter(planner_vars, "num_dofs", num_dofs_);
	    YAML::ReadParameter(planner_vars, "kd_iterations", kd_iterations_);
	    YAML::ReadParameter(planner_vars, "num_subsamples", num_subsamples_);
        YAML::ReadParameter(planner_vars, "display_motion", display_motion_);
	    YAML::ReadParameter(planner_vars, "min_joint_limits", min_joint_limits_);
        YAML::ReadParameter(planner_vars, "max_joint_limits", max_joint_limits_);
        YAML::ReadParameter(planner_vars, "default_joint_positions", default_joint_positions_);

        YAML::ReadParameter(planner_vars, "active_dofs", active_dofs_);
        extended_active_dofs_ = Eigen::VectorXi(active_dofs_.size()+6).setZero();
        for (int dof_id=0; dof_id<6; dof_id++) { extended_active_dofs_[dof_id] = dof_id; }
        for (int dof_id=0; dof_id<active_dofs_.size(); dof_id++) { extended_active_dofs_[6+dof_id] = 6+active_dofs_[dof_id]; }
      }

    //

	  // Dynamics parameters
	  YAML::ReadParameter(planner_vars, "num_com_viapoints", num_com_viapoints_);
	  com_viapoints_.clear();
	  for (int via_id=0; via_id<num_com_viapoints_; via_id++) {
	    com_viapoints_.push_back(Eigen::Vector4d::Zero());
	    YAML::ReadParameter(planner_vars["com_viapoints"], "via"+std::to_string(via_id), com_viapoints_[via_id]);
	  }
	  YAML::ReadParameter(planner_vars, "time_step", time_step_);
	  YAML::ReadParameter(planner_vars, "n_act_eefs", num_act_eefs_);
	  YAML::ReadParameter(planner_vars, "time_horizon", time_horizon_);
      YAML::ReadParameter(planner_vars, "min_rel_height", min_rel_height_);
	  YAML::ReadParameter(planner_vars, "external_force", external_force_);
      YAML::ReadParameter(planner_vars, "com_displacement", com_displacement_);
      if (YAML::ReadParameter<std::string>(planner_vars, "heuristic").compare("TrustRegion")==0) { heuristic_ = Heuristic::TrustRegion; }
      else if (YAML::ReadParameter<std::string>(planner_vars, "heuristic").compare("SoftConstraint")==0) { heuristic_ = Heuristic::SoftConstraint; }
      else if (YAML::ReadParameter<std::string>(planner_vars, "heuristic").compare("TimeOptimization")==0) { heuristic_ = Heuristic::TimeOptimization; }
      else { heuristic_ = Heuristic::SoftConstraint; }

      // Time optimization parameters
      if (heuristic_ == Heuristic::TimeOptimization) {
        YAML::ReadParameter(planner_vars, "max_time_iterations", max_time_iterations_);
        YAML::ReadParameter(planner_vars, "max_time_residual_tolerance", max_time_residual_tolerance_);
        YAML::ReadParameter(planner_vars, "min_time_residual_improvement", min_time_residual_improvement_);
      }

      // Configuration parameters
      YAML::ReadParameter(planner_vars, "gravity", gravity_);
      YAML::ReadParameter(planner_vars, "robot_mass", robot_mass_);
      YAML::ReadParameter(planner_vars, "floor_height", floor_height_);
      YAML::ReadParameter(planner_vars, "torque_range", torque_range_);
      YAML::ReadParameter(planner_vars, "friction_coeff", friction_coeff_);
      YAML::ReadParameter(planner_vars, "max_eef_lengths", max_eef_lengths_);
      if (heuristic_ == Heuristic::TimeOptimization) {
        YAML::ReadParameter(planner_vars, "time_range", time_range_);
        YAML::ReadParameter(planner_vars, "is_time_horizon_fixed", is_time_horizon_fixed_);
      }
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
    	    YAML::ReadParameter(planner_vars, "cop_range_"+Problem::idToEndeffectorString(eff_id), cop_range_[eff_id]);
    	    YAML::ReadParameter(planner_vars, "eff_offset_"+Problem::idToEndeffectorString(eff_id), eff_offset_[eff_id]);
      }
      is_friction_cone_linear_= (YAML::ReadParameter<std::string>(planner_vars, "friction_cone").compare("LinearCone")==0);

      // Dynamics weights
      YAML::ReadParameter(planner_vars, "w_com", w_com_);
      YAML::ReadParameter(planner_vars, "w_amom", w_amom_);
      YAML::ReadParameter(planner_vars, "w_lmom", w_lmom_);
      YAML::ReadParameter(planner_vars, "w_amomd", w_amomd_);
      YAML::ReadParameter(planner_vars, "w_lmomd", w_lmomd_);
      YAML::ReadParameter(planner_vars, "w_com_via", w_com_via_);
      YAML::ReadParameter(planner_vars, "w_trq_arm", w_trq_arm_);
      YAML::ReadParameter(planner_vars, "w_trq_leg", w_trq_leg_);
      YAML::ReadParameter(planner_vars, "w_frc_arm", w_frc_arm_);
      YAML::ReadParameter(planner_vars, "w_frc_leg", w_frc_leg_);
      YAML::ReadParameter(planner_vars, "w_dfrc_arm", w_dfrc_arm_);
      YAML::ReadParameter(planner_vars, "w_dfrc_leg", w_dfrc_leg_);
      YAML::ReadParameter(planner_vars, "w_amom_final", w_amom_final_);
      YAML::ReadParameter(planner_vars, "w_lmom_final", w_lmom_final_);
      YAML::ReadParameter(planner_vars, "w_com_track" , w_com_track_ );
      YAML::ReadParameter(planner_vars, "w_lmom_track", w_lmom_track_);
      YAML::ReadParameter(planner_vars, "w_amom_track", w_amom_track_);

      if (heuristic_ == Heuristic::TimeOptimization) {
        YAML::ReadParameter(planner_vars, "w_time", w_time_);
        YAML::ReadParameter(planner_vars, "w_time_penalty", w_time_penalty_);
      }

      if (load_kinematics_) {
        // Kinematics solver c++ paramters
	      YAML::ReadParameter(planner_vars, "w_kin_com", w_kin_com_);
	      YAML::ReadParameter(planner_vars, "w_kin_lmom", w_kin_lmom_);
	      YAML::ReadParameter(planner_vars, "w_kin_amom", w_kin_amom_);
	      YAML::ReadParameter(planner_vars, "w_kin_lmomd", w_kin_lmomd_);
	      YAML::ReadParameter(planner_vars, "w_kin_amomd", w_kin_amomd_);
	      YAML::ReadParameter(planner_vars, "w_kin_eff_pos", w_kin_eff_pos_);
	      YAML::ReadParameter(planner_vars, "w_kin_base_ori", w_kin_base_ori_);
	      YAML::ReadParameter(planner_vars, "w_kin_joint_vel", w_kin_joint_vel_);
	      YAML::ReadParameter(planner_vars, "w_kin_joint_acc", w_kin_joint_acc_);
        YAML::ReadParameter(planner_vars, "slacks_penalty", kin_slacks_penalty_);
	      YAML::ReadParameter(planner_vars, "integration_step", kin_integration_step_);
	      YAML::ReadParameter(planner_vars, "w_kin_eff_pos_nonact", w_kin_eff_pos_nonact_);
	      YAML::ReadParameter(planner_vars, "w_kin_default_joints", w_kin_default_joints_);
	      YAML::ReadParameter(planner_vars, "max_trajectory_iters", max_trajectory_iters_);
	      YAML::ReadParameter(planner_vars, "max_convergence_iters", max_convergence_iters_);
	      YAML::ReadParameter(planner_vars, "convergence_tolerance", convergence_tolerance_);
        YAML::ReadParameter(planner_vars, "lambda_regularization", lambda_regularization_);
        // IK version to be used in python
        YAML::ReadParameter(planner_vars, "inv_kin_solver", inv_kin_solver_);

        if (inv_kin_solver_ == 1) {
          // Kinematic paramters of first order IK (python)
          YAML::ReadParameter(planner_vars, "swing_traj_via_z", swing_traj_via_z_);
          YAML::ReadParameter(planner_vars, "w_lin_mom_tracking", w_lin_mom_tracking_);
          YAML::ReadParameter(planner_vars, "w_ang_mom_tracking", w_ang_mom_tracking_);
          YAML::ReadParameter(planner_vars, "w_endeff_contact", w_endeff_contact_);
          YAML::ReadParameter(planner_vars, "w_endeff_tracking", w_endeff_tracking_);
          YAML::ReadParameter(planner_vars, "p_endeff_tracking", p_endeff_tracking_);
          YAML::ReadParameter(planner_vars, "p_com_tracking", p_com_tracking_);
          YAML::ReadParameter(planner_vars, "w_joint_regularization", w_joint_regularization_);
          YAML::ReadParameter(planner_vars, "reg_orientation", reg_orientation_);
          YAML::ReadParameter(planner_vars, "reg_joint_position", reg_joint_position_);
          YAML::ReadParameter(planner_vars, "num_joint_viapoints", num_joint_viapoints_);
          YAML::ReadParameter(planner_vars, "num_base_viapoints", num_base_viapoints_);
        }
        else if (inv_kin_solver_ == 2){
          // Kinematic paramters of second order IK (python)
          YAML::ReadParameter(planner_vars, "swing_traj_via_z_second", swing_traj_via_z_second_);
          YAML::ReadParameter(planner_vars, "stance_weight", w_lin_mom_tracking_second_);
          YAML::ReadParameter(planner_vars, "w_ang_mom_tracking_second", w_ang_mom_tracking_second_);
          YAML::ReadParameter(planner_vars, "w_endeff_contact_second", w_endeff_contact_second_);
          YAML::ReadParameter(planner_vars, "w_endeff_tracking_second", w_endeff_tracking_second_);
          YAML::ReadParameter(planner_vars, "p_endeff_tracking_second", p_endeff_tracking_second_);
          YAML::ReadParameter(planner_vars, "p_com_tracking_second", p_com_tracking_second_);
          YAML::ReadParameter(planner_vars, "w_joint_regularization_second", w_joint_regularization_second_);
          YAML::ReadParameter(planner_vars, "d_endeff_tracking_second", d_endeff_tracking_second_);
          YAML::ReadParameter(planner_vars, "p_orient_tracking_second", p_orient_tracking_second_);
          YAML::ReadParameter(planner_vars, "d_orient_tracking_second", d_orient_tracking_second_);
          YAML::ReadParameter(planner_vars, "p_joint_regularization_second", p_joint_regularization_second_);
          YAML::ReadParameter(planner_vars, "d_joint_regularization_second", d_joint_regularization_second_);
          YAML::ReadParameter(planner_vars, "p_mom_tracking_second", p_mom_tracking_second_);
          YAML::ReadParameter(planner_vars, "num_joint_viapoints_second", num_joint_viapoints_second_);
          YAML::ReadParameter(planner_vars, "num_base_viapoints_second", num_base_viapoints_second_);
        }
        else if (inv_kin_solver_ == 3){
          // Kinematic paramters of nonlinear IK (python)
          YAML::ReadParameter(planner_vars, "swing_traj_via_z_nonlinear", swing_traj_via_z_nonlinear_);
          YAML::ReadParameter(planner_vars, "stance_weight_nonlinear", stance_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "swing_trackig_weight_nonlinear", swing_trackig_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "com_tracking_weight_nonlinear", com_tracking_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "momentum_tracking_weight_nonlinear", momentum_tracking_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "base_pos_reg_ratio_nonlinear", base_pos_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "base_ori_reg_ratio_nonlinear", base_ori_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "joint_pos_reg_ratio_nonlinear", joint_pos_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "base_vel_reg_ratio_nonlinear", base_vel_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "base_ang_vel_reg_ratio_nonlinear", base_ang_vel_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "joint_vel_reg_ratio_nonlinear", joint_vel_reg_ratio_nonlinear_);
          YAML::ReadParameter(planner_vars, "state_reg_weight_nonlinear", state_reg_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "control_reg_weight_nonlinear", control_reg_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "state_terminal_weight_nonlinear", state_terminal_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "control_terminal_weight_nonlinear", control_terminal_weight_nonlinear_);
          YAML::ReadParameter(planner_vars, "num_joint_viapoints_nonlinear", num_joint_viapoints_nonlinear_);
          YAML::ReadParameter(planner_vars, "num_base_viapoints_nonlinear", num_base_viapoints_nonlinear_);


        }
        // else{
        //   throw std::runtime_error("inv_kin_solver is invalid"); break;
        // }

        if (inv_kin_solver_ == 1) {
          joint_viapoints_.clear();
          base_viapoints_.clear();
        }
        else if (inv_kin_solver_ == 2) {
          joint_viapoints_second_.clear();
          base_viapoints_second_.clear();
        }
        else{
          joint_viapoints_nonlinear_.clear();
          base_viapoints_nonlinear_.clear();
        }
        if (inv_kin_solver_ == 1) {
          for (int via_id=0; via_id<num_joint_viapoints_; via_id++) {
            joint_viapoints_.push_back(Eigen::VectorXd::Zero(num_dofs_+1));
            YAML::ReadParameter(planner_vars["joint_viapoints"], "via"+std::to_string(via_id), joint_viapoints_[via_id]);
          }
            for (int via_id=0; via_id<num_base_viapoints_; via_id++) {
              base_viapoints_.push_back(Eigen::VectorXd::Zero(4));
              YAML::ReadParameter(planner_vars["base_viapoints"], "via"+std::to_string(via_id), base_viapoints_[via_id]);
          }
        }
        else if (inv_kin_solver_ == 2) {
          for (int via_id=0; via_id<num_joint_viapoints_second_; via_id++) {
            joint_viapoints_second_.push_back(Eigen::VectorXd::Zero(num_dofs_+1));
            YAML::ReadParameter(planner_vars["joint_viapoints_second"], "via"+std::to_string(via_id), joint_viapoints_second_[via_id]);
          }
          for (int via_id=0; via_id<num_base_viapoints_second_; via_id++) {
            base_viapoints_second_.push_back(Eigen::VectorXd::Zero(4));
            YAML::ReadParameter(planner_vars["base_viapoints_second"], "via"+std::to_string(via_id), base_viapoints_second_[via_id]);
          }
        }
        else{
          for (int via_id=0; via_id<num_joint_viapoints_nonlinear_; via_id++) {
            joint_viapoints_second_.push_back(Eigen::VectorXd::Zero(num_dofs_+1));
            YAML::ReadParameter(planner_vars["joint_viapoints_second"], "via"+std::to_string(via_id), joint_viapoints_nonlinear_[via_id]);
          }
          for (int via_id=0; via_id<num_base_viapoints_nonlinear_; via_id++) {
            base_viapoints_second_.push_back(Eigen::VectorXd::Zero(4));
            YAML::ReadParameter(planner_vars["base_viapoints_second"], "via"+std::to_string(via_id), base_viapoints_nonlinear_[via_id]);
          }
        }
      }

      // Storage information
      YAML::ReadParameter(planner_vars, "store_data", store_data_);

      // Solver setting
      YAML::ReadParameter(planner_vars, "use_default_solver_setting", use_default_solver_setting_);

      num_act_dofs_ = active_dofs_.size();
      mass_times_gravity_ = robot_mass_ * gravity_;
      gravity_vector_ = Eigen::Vector3d(0., 0., -gravity_);
      num_timesteps_ = std::floor(time_horizon_/time_step_);
      num_extended_act_dofs_ = extended_active_dofs_.size();
    }
    catch (std::runtime_error& e)
    {
      std::cout << "From ["<< __FILE__"]: "
                << "Error while loading the YAML file [" + cfg_file + "]."
                << "Error message is: " << e.what() << std::endl << std::endl;
    }
  }

  const int& PlannerSetting::get(PlannerIntParam param) const
  {
    switch (param)
    {
      // Kinematics parameters
      case PlannerIntParam_NumDofs : { return num_dofs_; }
      case PlannerIntParam_NumActiveDofs : { return num_act_dofs_; }
      case PlannerIntParam_NumSubsamples : { return num_subsamples_; }
      case PlannerIntParam_KinDynIterations : { return kd_iterations_; }
      case PlannerIntParam_NumExtendedActiveDofs : { return num_extended_act_dofs_; }
      case PlannerIntParam_MaxKinTrajectoryIterations : { return max_trajectory_iters_; }
      case PlannerIntParam_MaxKinConvergenceIterations : { return max_convergence_iters_; }

      // Dynamics parameters
      case PlannerIntParam_NumTimesteps : { return num_timesteps_; }
      case PlannerIntParam_NumViapoints : { return num_com_viapoints_; }
      case PlannerIntParam_NumActiveEndeffectors : { return num_act_eefs_; }

      // First order IK (python) parameters
      case PlannerIntParam_NumJointViapoints : { return num_joint_viapoints_; }
      case PlannerIntParam_NumBaseViapoints : { return num_base_viapoints_; }

      // Second order IK (python) parameters
      case PlannerIntParam_NumJointViapoints_Second : { return num_joint_viapoints_second_; }
      case PlannerIntParam_NumBaseViapoints_Second : { return num_base_viapoints_second_; }

      // Nonlinear IK (python) parameters
      case PlannerIntParam_NumJointViapoints_Nonlinear : { return num_joint_viapoints_nonlinear_; }
      case PlannerIntParam_NumBaseViapoints_Nonlinear : { return num_base_viapoints_nonlinear_; }

      // Inverse kinematics (python)
      case PlannerIntParam_InverseKinematicsSolver : { return inv_kin_solver_; }

      // Time optimization parameters
      case PlannerIntParam_MaxNumTimeIterations : { return max_time_iterations_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerIntParam invalid"); break; }
    }
  }

  const bool& PlannerSetting::get(PlannerBoolParam param) const
  {
    switch (param)
    {
      // Configuration parameters
      case PlannerBoolParam_IsTimeHorizonFixed : { return is_time_horizon_fixed_; }
      case PlannerBoolParam_IsFrictionConeLinear : { return is_friction_cone_linear_; }

      // Kinematics parameters
      case PlannerBoolParam_DisplayMotion : { return display_motion_; }
      case PlannerBoolParam_LoadKinematics : { return load_kinematics_; }

      // Storage information
      case PlannerBoolParam_StoreData : { return store_data_; }

      // Solver setting
      case PlannerBoolParam_UseDefaultSolverSetting : { return use_default_solver_setting_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerBoolParam invalid"); break; }
    }
  }

  const double& PlannerSetting::get(PlannerDoubleParam param) const
  {
    switch (param)
    {
      // Kinematics parameters
      case PlannerDoubleParam_KinSlacksPenalty : { return kin_slacks_penalty_; }
      case PlannerDoubleParam_KinIntegrationStep : { return kin_integration_step_; }
      case PlannerDoubleParam_LambdaRegularization : { return lambda_regularization_; }
      case PlannerDoubleParam_KinConvergenceTolerance : { return convergence_tolerance_; }

      // Dynamics parameters
      case PlannerDoubleParam_TimeStep : { return time_step_; }
      case PlannerDoubleParam_TimeHorizon : { return time_horizon_; }
      case PlannerDoubleParam_MinRelHeight : { return min_rel_height_; }

      // Time optimization parameters
      case PlannerDoubleParam_MaxTimeResidualTolerance : { return max_time_residual_tolerance_; }
      case PlannerDoubleParam_MinTimeResidualImprovement : { return min_time_residual_improvement_; }

      // Configuration parameters
      case PlannerDoubleParam_Gravity : { return gravity_; }
      case PlannerDoubleParam_RobotMass : { return robot_mass_; }
      case PlannerDoubleParam_FloorHeight : { return floor_height_; }
      case PlannerDoubleParam_RobotWeight : { return mass_times_gravity_; }
      case PlannerDoubleParam_FrictionCoefficient : { return friction_coeff_; }
      case PlannerDoubleParam_MassTimesGravity : { return mass_times_gravity_; }

      // Dynamics optimization weights
      case PlannerDoubleParam_WeightArmTorque : { return w_trq_arm_; }
      case PlannerDoubleParam_WeightLegTorque : { return w_trq_leg_; }
      case PlannerDoubleParam_WeightTimeRegularization : { return w_time_;}
      case PlannerDoubleParam_WeightTimePenalty : { return w_time_penalty_; }

      // weights of first order IK (python)
      case PlannerDoubleParam_SwingTrajViaZ : { return swing_traj_via_z_; }
      case PlannerDoubleParam_WeightLinMomentumTracking : { return w_lin_mom_tracking_; }
      case PlannerDoubleParam_WeightAngMomentumTracking : { return w_ang_mom_tracking_; }
      case PlannerDoubleParam_WeightEndEffContact : { return w_endeff_contact_; }
      case PlannerDoubleParam_WeightEndEffTracking : { return w_endeff_tracking_; }
      case PlannerDoubleParam_PGainEndEffTracking : { return p_endeff_tracking_; }
      case PlannerDoubleParam_PGainComTracking : { return p_com_tracking_; }
      case PlannerDoubleParam_WeightJointReg : { return w_joint_regularization_; }
      case PlannerDoubleParam_PGainOrientationTracking : { return reg_orientation_; }
      case PlannerDoubleParam_PGainPositionTracking : { return reg_joint_position_; }

      // weights of second order IK (python)
      case PlannerDoubleParam_SwingTrajViaZ_Second : { return swing_traj_via_z_second_; }
      case PlannerDoubleParam_WeightLinMomentumTracking_Second : { return w_lin_mom_tracking_second_; }
      case PlannerDoubleParam_WeightAngMomentumTracking_Second : { return w_ang_mom_tracking_second_; }
      case PlannerDoubleParam_WeightEndEffContact_Second : { return w_endeff_contact_second_; }
      case PlannerDoubleParam_WeightEndEffTracking_Second : { return w_endeff_tracking_second_; }
      case PlannerDoubleParam_PGainEndEffTracking_Second : { return p_endeff_tracking_second_; }
      case PlannerDoubleParam_PGainComTracking_Second : { return p_com_tracking_second_; }
      case PlannerDoubleParam_WeightJointReg_Second : { return w_joint_regularization_second_; }
      case PlannerDoubleParam_DGainEndEffTracking_Second : { return d_endeff_tracking_second_; }
      case PlannerDoubleParam_PGainBaseOrientationTracking_Second : { return p_orient_tracking_second_; }
      case PlannerDoubleParam_DGainBaseOrientationTracking_Second : { return d_orient_tracking_second_; }
      case PlannerDoubleParam_PGainJointRegularization_Second : { return p_joint_regularization_second_; }
      case PlannerDoubleParam_DGainJointRegularization_Second : { return d_joint_regularization_second_; }

      // weights of Nonlinear IK (python)
      case PlannerDoubleParam_SwingTrajViaZ_Nonlinear : { return swing_traj_via_z_nonlinear_; }
      case PlannerDoubleParam_WeightStance_Nonlinear : { return stance_weight_nonlinear_; }
      case PlannerDoubleParam_WeightSwingTracking_Nonlinear : { return swing_trackig_weight_nonlinear_; }
      case PlannerDoubleParam_WeightComTracking_Nonlinear : { return com_tracking_weight_nonlinear_; }
      case PlannerDoubleParam_WeightMomentumTracking_Nonlinear : { return momentum_tracking_weight_nonlinear_; }
      case PlannerDoubleParam_RatioBasePosReg_Nonlinear : { return base_pos_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseOriReg_Nonlinear : { return base_ori_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioJointPosReg_Nonlinear : { return joint_pos_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseVelReg_Nonlinear : { return base_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseAngVelReg_Nonlinear : { return base_ang_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioJointVelReg_Nonlinear : { return joint_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_WeightStateReg_Nonlinear : { return state_reg_weight_nonlinear_; }
      case PlannerDoubleParam_WeightControlReg_Nonlinear : { return control_reg_weight_nonlinear_; }
      case PlannerDoubleParam_WeightTerminalStateReg_Nonlinear : { return state_terminal_weight_nonlinear_; }
      case PlannerDoubleParam_WeightTerminalControlReg_Nonlinear : { return control_terminal_weight_nonlinear_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerDoubleParam invalid"); break; }
    }
  }

  const std::string& PlannerSetting::get(PlannerStringParam param) const
  {
    switch (param)
	{
      // Storage information
      case PlannerStringParam_ConfigFile : { return cfg_file_; }
      case PlannerStringParam_SaveDynamicsFile : { return save_dynamics_file_; }
      case PlannerStringParam_SaveKinematicsFile : { return save_kinematics_file_; }

      // Solver setting
      case PlannerStringParam_DefaultSolverSettingFile : { return default_solver_setting_file_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerStringParam invalid"); break; }
	}
  }

  const Eigen::Ref<const Eigen::VectorXi> PlannerSetting::get(PlannerIntVectorParam param) const
  {
    switch (param)
	{
      // Kinematics parameters
      case PlannerIntVectorParam_ActiveDofs : { return active_dofs_; }
      case PlannerIntVectorParam_ExtendedActiveDofs : { return extended_active_dofs_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerVectorParam invalid"); break; }
	}
  }

  const Eigen::Ref<const Eigen::VectorXd> PlannerSetting::get(PlannerVectorParam param) const
  {
    switch (param)
	{
      // Dynamics parameters
      case PlannerVectorParam_ExternalForce : { return external_force_; }
      case PlannerVectorParam_CenterOfMassMotion : { return com_displacement_; }

      // Kinematics parameters
      case PlannerVectorParam_MinJointAngles : { return min_joint_limits_; }
      case PlannerVectorParam_MaxJointAngles : { return max_joint_limits_; }
      case PlannerVectorParam_KinematicDefaultJointPositions: { return default_joint_positions_; }

      // Configuration parameters
      case PlannerVectorParam_TimeRange : { return time_range_; }
      case PlannerVectorParam_TorqueRange : { return torque_range_; }
      case PlannerVectorParam_GravityVector : { return gravity_vector_; }
      case PlannerVectorParam_MaxEndeffectorLengths : { return max_eef_lengths_; }

      // Dynamics optimization weights
      case PlannerVectorParam_WeightArmForce : { return w_frc_arm_; }
      case PlannerVectorParam_WeightLegForce : { return w_frc_leg_; }
      case PlannerVectorParam_WeightCenterOfMass : { return w_com_; }
      case PlannerVectorParam_WeightLinearMomentum : { return w_lmom_; }
      case PlannerVectorParam_WeightAngularMomentum : { return w_amom_; }
      case PlannerVectorParam_WeightArmForceRate : { return w_dfrc_arm_; }
      case PlannerVectorParam_WeightLegForceRate : { return w_dfrc_leg_; }
      case PlannerVectorParam_WeightLinearMomentumRate : { return w_lmomd_; }
      case PlannerVectorParam_WeightAngularMomentumRate : { return w_amomd_; }
      case PlannerVectorParam_WeightCenterOfMassViapoint : { return w_com_via_; }
      case PlannerVectorParam_WeightFinalLinearMomentum : { return w_lmom_final_; }
      case PlannerVectorParam_WeightFinalAngularMomentum : { return w_amom_final_; }
      case PlannerVectorParam_WeightDynamicTrackingCenterOfMass : { return w_com_track_; }
      case PlannerVectorParam_WeightDynamicTrackingLinearMomentum : { return w_lmom_track_; }
      case PlannerVectorParam_WeightDynamicTrackingAngularMomentum : { return w_amom_track_; }

      // Kinematics optimization weights
      case PlannerVectorParam_WeightJointVelocity : { return w_kin_joint_vel_; }
      case PlannerVectorParam_WeightJointAcceleration : { return w_kin_joint_acc_; }
      case PlannerVectorParam_WeightKinematicTrackingCenterOfMass : { return w_kin_com_; }
      case PlannerVectorParam_WeightKinematicTrackingLinearMomentum : { return w_kin_lmom_; }
      case PlannerVectorParam_WeightKinematicTrackingAngularMomentum : { return w_kin_amom_; }
      case PlannerVectorParam_WeightKinematicTrackingBaseOrientation : { return w_kin_base_ori_; }
      case PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate : { return w_kin_lmomd_; }
      case PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate : { return w_kin_amomd_; }
      case PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition : { return w_kin_eff_pos_; }
      case PlannerVectorParam_WeightKinematicDefaultJointPositions: { return w_kin_default_joints_; }
      case PlannerVectorParam_WeightKinematicTrackingNonActiveEndeffectorPosition : { return w_kin_eff_pos_nonact_; }

      //Second order IK (python)
      case PlannerVectorParam_PGainMomentumTracking_Second : { return p_mom_tracking_second_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerVectorParam invalid"); break; }
	}
  }

  const std::array<Eigen::VectorXd, Problem::n_endeffs_>& PlannerSetting::get(PlannerArrayParam param) const
  {
    switch (param)
	{
    // Configuration parameters
    case PlannerArrayParam_EndeffectorOffset : { return eff_offset_; }
    case PlannerArrayParam_CenterOfPressureRange : { return cop_range_; }

    // Not handled parameters
    default: { throw std::runtime_error("PlannerSetting::get PlannerVectorParam invalid"); break; }
	}
  }

  const std::vector<Eigen::VectorXd>& PlannerSetting::get(PlannerCVectorParam param) const
  {
    switch (param)
	{
    // Configuration parameters
    case PlannerCVectorParam_Viapoints : { return com_viapoints_; }
    case PlannerCVectorParam_JointViapoints : { return joint_viapoints_; }
    case PlannerCVectorParam_BaseViapoints : { return base_viapoints_; }
    case PlannerCVectorParam_JointViapoints_Second : { return joint_viapoints_second_; }
    case PlannerCVectorParam_BaseViapoints_Second : { return base_viapoints_second_; }
    case PlannerCVectorParam_JointViapoints_Nonlinear : { return joint_viapoints_nonlinear_; }
    case PlannerCVectorParam_BaseViapoints_Nonlinear : { return base_viapoints_nonlinear_; }

    // Not handled parameters
    default: { throw std::runtime_error("PlannerSetting::get PlannerCVectorParam invalid"); break; }
	}
  }

  int& PlannerSetting::get(PlannerIntParam param)
  {
    switch (param)
    {
      // Kinematics parameters
      case PlannerIntParam_NumDofs : { return num_dofs_; }
      case PlannerIntParam_NumActiveDofs : { return num_act_dofs_; }
      case PlannerIntParam_NumSubsamples : { return num_subsamples_; }
      case PlannerIntParam_KinDynIterations : { return kd_iterations_; }
      case PlannerIntParam_NumExtendedActiveDofs : { return num_extended_act_dofs_; }
      case PlannerIntParam_MaxKinTrajectoryIterations : { return max_trajectory_iters_; }
      case PlannerIntParam_MaxKinConvergenceIterations : { return max_convergence_iters_; }

      // Dynamics parameters
      case PlannerIntParam_NumTimesteps : { return num_timesteps_; }
      case PlannerIntParam_NumViapoints : { return num_com_viapoints_; }
      case PlannerIntParam_NumActiveEndeffectors : { return num_act_eefs_; }

      // First order IK (python) parameters
      case PlannerIntParam_NumJointViapoints : { return num_joint_viapoints_; }
      case PlannerIntParam_NumBaseViapoints : { return num_base_viapoints_; }

      // Second order IK (python) parameters
      case PlannerIntParam_NumJointViapoints_Second : { return num_joint_viapoints_second_; }
      case PlannerIntParam_NumBaseViapoints_Second : { return num_base_viapoints_second_; }

      // Nonlinear IK (python) parameters
      case PlannerIntParam_NumJointViapoints_Nonlinear : { return num_joint_viapoints_nonlinear_; }
      case PlannerIntParam_NumBaseViapoints_Nonlinear : { return num_base_viapoints_nonlinear_; }

      // Inverse kinematics (python)
      case PlannerIntParam_InverseKinematicsSolver : { return inv_kin_solver_; }

      // Time optimization parameters
      case PlannerIntParam_MaxNumTimeIterations : { return max_time_iterations_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerIntParam invalid"); break; }
    }
  }

  bool& PlannerSetting::get(PlannerBoolParam param)
  {
    switch (param)
    {
      // Configuration parameters
      case PlannerBoolParam_IsTimeHorizonFixed : { return is_time_horizon_fixed_; }
      case PlannerBoolParam_IsFrictionConeLinear : { return is_friction_cone_linear_; }

      // Kinematics parameters
      case PlannerBoolParam_DisplayMotion : { return display_motion_; }
      case PlannerBoolParam_LoadKinematics : { return load_kinematics_; }

      // Storage information
      case PlannerBoolParam_StoreData : { return store_data_; }

      // Solver setting
      case PlannerBoolParam_UseDefaultSolverSetting : { return use_default_solver_setting_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::set PlannerBoolParam invalid"); break; }
    }
  }

  double& PlannerSetting::get(PlannerDoubleParam param)
  {
    switch (param)
    {
      // Kinematics parameters
      case PlannerDoubleParam_KinSlacksPenalty : { return kin_slacks_penalty_; }
      case PlannerDoubleParam_KinIntegrationStep : { return kin_integration_step_; }
      case PlannerDoubleParam_LambdaRegularization : { return lambda_regularization_; }
      case PlannerDoubleParam_KinConvergenceTolerance : { return convergence_tolerance_; }

      // Dynamics parameters
      case PlannerDoubleParam_TimeStep : { return time_step_; }
      case PlannerDoubleParam_TimeHorizon : { return time_horizon_; }
      case PlannerDoubleParam_MinRelHeight : { return min_rel_height_; }

      // Time optimization parameters
      case PlannerDoubleParam_MaxTimeResidualTolerance : { return max_time_residual_tolerance_; }
      case PlannerDoubleParam_MinTimeResidualImprovement : { return min_time_residual_improvement_; }

      // Configuration parameters
      case PlannerDoubleParam_Gravity : { return gravity_; }
      case PlannerDoubleParam_RobotMass : { return robot_mass_; }
      case PlannerDoubleParam_FloorHeight : { return floor_height_; }
      case PlannerDoubleParam_RobotWeight : { return mass_times_gravity_; }
      case PlannerDoubleParam_FrictionCoefficient : { return friction_coeff_; }
      case PlannerDoubleParam_MassTimesGravity : { return mass_times_gravity_; }

      // Dynamics optimization weights
      case PlannerDoubleParam_WeightArmTorque : { return w_trq_arm_; }
      case PlannerDoubleParam_WeightLegTorque : { return w_trq_leg_; }
      case PlannerDoubleParam_WeightTimeRegularization : { return w_time_;}
      case PlannerDoubleParam_WeightTimePenalty : { return w_time_penalty_; }

      // weights of first order IK (python)
      case PlannerDoubleParam_SwingTrajViaZ : { return swing_traj_via_z_; }
      case PlannerDoubleParam_WeightLinMomentumTracking : { return w_lin_mom_tracking_; }
      case PlannerDoubleParam_WeightAngMomentumTracking : { return w_ang_mom_tracking_; }
      case PlannerDoubleParam_WeightEndEffContact : { return w_endeff_contact_; }
      case PlannerDoubleParam_WeightEndEffTracking : { return w_endeff_tracking_; }
      case PlannerDoubleParam_PGainEndEffTracking : { return p_endeff_tracking_; }
      case PlannerDoubleParam_PGainComTracking : { return p_com_tracking_; }
      case PlannerDoubleParam_WeightJointReg : { return w_joint_regularization_; }
      case PlannerDoubleParam_PGainOrientationTracking : { return reg_orientation_; }
      case PlannerDoubleParam_PGainPositionTracking : { return reg_joint_position_; }

      // weights of second order IK (python)
      case PlannerDoubleParam_SwingTrajViaZ_Second : { return swing_traj_via_z_second_; }
      case PlannerDoubleParam_WeightLinMomentumTracking_Second : { return w_lin_mom_tracking_second_; }
      case PlannerDoubleParam_WeightAngMomentumTracking_Second : { return w_ang_mom_tracking_second_; }
      case PlannerDoubleParam_WeightEndEffContact_Second : { return w_endeff_contact_second_; }
      case PlannerDoubleParam_WeightEndEffTracking_Second : { return w_endeff_tracking_second_; }
      case PlannerDoubleParam_PGainEndEffTracking_Second : { return p_endeff_tracking_second_; }
      case PlannerDoubleParam_PGainComTracking_Second : { return p_com_tracking_second_; }

      // weights of nonlinear IK (python)
      case PlannerDoubleParam_SwingTrajViaZ_Nonlinear : { return swing_traj_via_z_nonlinear_; }
      case PlannerDoubleParam_WeightStance_Nonlinear : { return stance_weight_nonlinear_; }
      case PlannerDoubleParam_WeightSwingTracking_Nonlinear : { return swing_trackig_weight_nonlinear_; }
      case PlannerDoubleParam_WeightComTracking_Nonlinear : { return com_tracking_weight_nonlinear_; }
      case PlannerDoubleParam_WeightMomentumTracking_Nonlinear : { return momentum_tracking_weight_nonlinear_; }
      case PlannerDoubleParam_RatioBasePosReg_Nonlinear : { return base_pos_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseOriReg_Nonlinear : { return base_ori_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioJointPosReg_Nonlinear : { return joint_pos_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseVelReg_Nonlinear : { return base_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioBaseAngVelReg_Nonlinear : { return base_ang_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_RatioJointVelReg_Nonlinear : { return joint_vel_reg_ratio_nonlinear_; }
      case PlannerDoubleParam_WeightStateReg_Nonlinear : { return state_reg_weight_nonlinear_; }
      case PlannerDoubleParam_WeightControlReg_Nonlinear : { return control_reg_weight_nonlinear_; }
      case PlannerDoubleParam_WeightTerminalStateReg_Nonlinear : { return state_terminal_weight_nonlinear_; }
      case PlannerDoubleParam_WeightTerminalControlReg_Nonlinear : { return control_terminal_weight_nonlinear_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::set PlannerDoubleParam invalid"); break; }
    }
  }

  std::string& PlannerSetting::get(PlannerStringParam param)
  {
    switch (param)
	{
      // Storage information
      case PlannerStringParam_ConfigFile : { return cfg_file_; }
      case PlannerStringParam_SaveDynamicsFile : { return save_dynamics_file_; }
      case PlannerStringParam_SaveKinematicsFile : { return save_kinematics_file_; }

      // Solver setting
      case PlannerStringParam_DefaultSolverSettingFile : { return default_solver_setting_file_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerStringParam invalid"); break; }
	}
  }

  Eigen::Ref<Eigen::VectorXi> PlannerSetting::get(PlannerIntVectorParam param)
  {
    switch (param)
	{
      // Kinematics parameters
      case PlannerIntVectorParam_ActiveDofs : { return active_dofs_; }
      case PlannerIntVectorParam_ExtendedActiveDofs : { return extended_active_dofs_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerIntVectorParam invalid"); break; }
	}
  }

  Eigen::Ref<Eigen::VectorXd> PlannerSetting::get(PlannerVectorParam param)
  {
    switch (param)
	{
      // Dynamics parameters
      case PlannerVectorParam_ExternalForce : { return external_force_; }
      case PlannerVectorParam_CenterOfMassMotion : { return com_displacement_; }

      // Kinematics parameters
      case PlannerVectorParam_MinJointAngles : { return min_joint_limits_; }
      case PlannerVectorParam_MaxJointAngles : { return max_joint_limits_; }
      case PlannerVectorParam_KinematicDefaultJointPositions: { return default_joint_positions_; }

      // Configuration parameters
      case PlannerVectorParam_TimeRange : { return time_range_; }
      case PlannerVectorParam_TorqueRange : { return torque_range_; }
      case PlannerVectorParam_GravityVector : { return gravity_vector_; }
      case PlannerVectorParam_MaxEndeffectorLengths : { return max_eef_lengths_; }

      // Dynamics optimization weights
      case PlannerVectorParam_WeightArmForce : { return w_frc_arm_; }
      case PlannerVectorParam_WeightLegForce : { return w_frc_leg_; }
      case PlannerVectorParam_WeightCenterOfMass : { return w_com_; }
      case PlannerVectorParam_WeightLinearMomentum : { return w_lmom_; }
      case PlannerVectorParam_WeightAngularMomentum : { return w_amom_; }
      case PlannerVectorParam_WeightArmForceRate : { return w_dfrc_arm_; }
      case PlannerVectorParam_WeightLegForceRate : { return w_dfrc_leg_; }
      case PlannerVectorParam_WeightLinearMomentumRate : { return w_lmomd_; }
      case PlannerVectorParam_WeightAngularMomentumRate : { return w_amomd_; }
      case PlannerVectorParam_WeightCenterOfMassViapoint : { return w_com_via_; }
      case PlannerVectorParam_WeightFinalLinearMomentum : { return w_lmom_final_; }
      case PlannerVectorParam_WeightFinalAngularMomentum : { return w_amom_final_; }
      case PlannerVectorParam_WeightDynamicTrackingCenterOfMass : { return w_com_track_; }
      case PlannerVectorParam_WeightDynamicTrackingLinearMomentum : { return w_lmom_track_; }
      case PlannerVectorParam_WeightDynamicTrackingAngularMomentum : { return w_amom_track_; }

      // Kinematics optimization weights
      case PlannerVectorParam_WeightJointVelocity : { return w_kin_joint_vel_; }
      case PlannerVectorParam_WeightJointAcceleration : { return w_kin_joint_acc_; }
      case PlannerVectorParam_WeightKinematicTrackingCenterOfMass : { return w_kin_com_; }
      case PlannerVectorParam_WeightKinematicTrackingLinearMomentum : { return w_kin_lmom_; }
      case PlannerVectorParam_WeightKinematicTrackingAngularMomentum : { return w_kin_amom_; }
      case PlannerVectorParam_WeightKinematicTrackingBaseOrientation : { return w_kin_base_ori_; }
      case PlannerVectorParam_WeightKinematicTrackingLinearMomentumRate : { return w_kin_lmomd_; }
      case PlannerVectorParam_WeightKinematicTrackingAngularMomentumRate : { return w_kin_amomd_; }
      case PlannerVectorParam_WeightKinematicTrackingEndeffectorPosition : { return w_kin_eff_pos_; }
      case PlannerVectorParam_WeightKinematicDefaultJointPositions: { return w_kin_default_joints_; }
      case PlannerVectorParam_WeightKinematicTrackingNonActiveEndeffectorPosition : { return w_kin_eff_pos_nonact_; }

      //Second order IK (python)
      case PlannerVectorParam_PGainMomentumTracking_Second : { return p_mom_tracking_second_; }

      // Not handled parameters
      default: { throw std::runtime_error("PlannerSetting::get PlannerVectorParam invalid"); break; }
	}
  }

  std::array<Eigen::VectorXd, Problem::n_endeffs_>& PlannerSetting::get(PlannerArrayParam param)
  {
    switch (param)
	{
    // Configuration parameters
    case PlannerArrayParam_EndeffectorOffset : { return eff_offset_; }
    case PlannerArrayParam_CenterOfPressureRange : { return cop_range_; }

    // Not handled parameters
    default: { throw std::runtime_error("PlannerSetting::get PlannerVectorParam invalid"); break; }
	}
  }

  std::vector<Eigen::VectorXd>& PlannerSetting::get(PlannerCVectorParam param)
  {
    switch (param)
	{
    // Configuration parameters
    case PlannerCVectorParam_Viapoints : { return com_viapoints_; }
    case PlannerCVectorParam_JointViapoints : { return joint_viapoints_; }
    case PlannerCVectorParam_BaseViapoints : { return base_viapoints_; }
    case PlannerCVectorParam_JointViapoints_Second : { return joint_viapoints_second_; }
    case PlannerCVectorParam_BaseViapoints_Second : { return base_viapoints_second_; }
    case PlannerCVectorParam_JointViapoints_Nonlinear : { return joint_viapoints_nonlinear_; }
    case PlannerCVectorParam_BaseViapoints_Nonlinear : { return base_viapoints_nonlinear_; }


    // Not handled parameters
    default: { throw std::runtime_error("PlannerSetting::get PlannerCVectorParam invalid"); break; }
	}
  }

}
