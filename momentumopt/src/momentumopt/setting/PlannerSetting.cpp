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

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <momentumopt/setting/PlannerSetting.hpp>

namespace momentumopt {

  // PlannerSetting class function implementations
  void PlannerSetting::initialize(const std::string& cfg_file, const std::string& planner_vars_yaml)
  {
    cfg_file_ = cfg_file;
    save_dynamics_file_ = cfg_file.substr(0, cfg_file.size()-5) + "_dyn_results.yaml";
    save_kinematics_file_ = cfg_file.substr(0, cfg_file.size()-5) + "_kin_results.yaml";
    default_solver_setting_file_ = CFG_SRC_PATH + std::string("default_solver_setting.yaml");

    try
    {
	  YAML::Node planner_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node planner_vars = planner_cfg[planner_vars_yaml.c_str()];

	  // Kinematics parameters
	  readParameter(planner_vars, "load_kinematics", load_kinematics_);
      if (load_kinematics_) {
	    readParameter(planner_vars, "num_dofs", num_dofs_);
	    readParameter(planner_vars, "kd_iterations", kd_iterations_);
	    readParameter(planner_vars, "num_subsamples", num_subsamples_);
        readParameter(planner_vars, "display_motion", display_motion_);
	    readParameter(planner_vars, "min_joint_limits", min_joint_limits_);
        readParameter(planner_vars, "max_joint_limits", max_joint_limits_);
        readParameter(planner_vars, "default_joint_positions", default_joint_positions_);

        readParameter(planner_vars, "active_dofs", active_dofs_);
        extended_active_dofs_ = Eigen::VectorXi(active_dofs_.size()+6).setZero();
        for (int dof_id=0; dof_id<6; dof_id++) { extended_active_dofs_[dof_id] = dof_id; }
        for (int dof_id=0; dof_id<active_dofs_.size(); dof_id++) { extended_active_dofs_[6+dof_id] = 6+active_dofs_[dof_id]; }
      }

    //

	  // Dynamics parameters
	  readParameter(planner_vars, "num_com_viapoints", num_com_viapoints_);
	  com_viapoints_.clear();
	  for (int via_id=0; via_id<num_com_viapoints_; via_id++) {
	    com_viapoints_.push_back(Eigen::Vector4d::Zero());
	    readParameter(planner_vars["com_viapoints"], "via"+std::to_string(via_id), com_viapoints_[via_id]);
	  }
	  readParameter(planner_vars, "time_step", time_step_);
	  readParameter(planner_vars, "n_act_eefs", num_act_eefs_);
	  readParameter(planner_vars, "time_horizon", time_horizon_);
      readParameter(planner_vars, "min_rel_height", min_rel_height_);
	  readParameter(planner_vars, "external_force", external_force_);
      readParameter(planner_vars, "com_displacement", com_displacement_);
      if (readParameter<std::string>(planner_vars, "heuristic").compare("TrustRegion")==0) { heuristic_ = Heuristic::TrustRegion; }
      else if (readParameter<std::string>(planner_vars, "heuristic").compare("SoftConstraint")==0) { heuristic_ = Heuristic::SoftConstraint; }
      else if (readParameter<std::string>(planner_vars, "heuristic").compare("TimeOptimization")==0) { heuristic_ = Heuristic::TimeOptimization; }
      else { heuristic_ = Heuristic::SoftConstraint; }

      // Time optimization parameters
      if (heuristic_ == Heuristic::TimeOptimization) {
        readParameter(planner_vars, "max_time_iterations", max_time_iterations_);
        readParameter(planner_vars, "max_time_residual_tolerance", max_time_residual_tolerance_);
        readParameter(planner_vars, "min_time_residual_improvement", min_time_residual_improvement_);
      }

      // Configuration parameters
      readParameter(planner_vars, "gravity", gravity_);
      readParameter(planner_vars, "robot_mass", robot_mass_);
      readParameter(planner_vars, "floor_height", floor_height_);
      readParameter(planner_vars, "torque_range", torque_range_);
      readParameter(planner_vars, "friction_coeff", friction_coeff_);
      readParameter(planner_vars, "max_eef_lengths", max_eef_lengths_);
      if (heuristic_ == Heuristic::TimeOptimization) {
        readParameter(planner_vars, "time_range", time_range_);
        readParameter(planner_vars, "is_time_horizon_fixed", is_time_horizon_fixed_);
      }
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
    	    readParameter(planner_vars, "cop_range_"+Problem::idToEndeffectorString(eff_id), cop_range_[eff_id]);
    	    readParameter(planner_vars, "eff_offset_"+Problem::idToEndeffectorString(eff_id), eff_offset_[eff_id]);
      }
      is_friction_cone_linear_= (readParameter<std::string>(planner_vars, "friction_cone").compare("LinearCone")==0);

      // Dynamics weights
      readParameter(planner_vars, "w_com", w_com_);
      readParameter(planner_vars, "w_amom", w_amom_);
      readParameter(planner_vars, "w_lmom", w_lmom_);
      readParameter(planner_vars, "w_amomd", w_amomd_);
      readParameter(planner_vars, "w_lmomd", w_lmomd_);
      readParameter(planner_vars, "w_com_via", w_com_via_);
      readParameter(planner_vars, "w_trq_arm", w_trq_arm_);
      readParameter(planner_vars, "w_trq_leg", w_trq_leg_);
      readParameter(planner_vars, "w_frc_arm", w_frc_arm_);
      readParameter(planner_vars, "w_frc_leg", w_frc_leg_);
      readParameter(planner_vars, "w_dfrc_arm", w_dfrc_arm_);
      readParameter(planner_vars, "w_dfrc_leg", w_dfrc_leg_);
      readParameter(planner_vars, "w_amom_final", w_amom_final_);
      readParameter(planner_vars, "w_lmom_final", w_lmom_final_);
      readParameter(planner_vars, "w_com_track" , w_com_track_ );
      readParameter(planner_vars, "w_lmom_track", w_lmom_track_);
      readParameter(planner_vars, "w_amom_track", w_amom_track_);

      if (heuristic_ == Heuristic::TimeOptimization) {
        readParameter(planner_vars, "w_time", w_time_);
        readParameter(planner_vars, "w_time_penalty", w_time_penalty_);
      }

      // Kinematics weights
      if (load_kinematics_) {
	    readParameter(planner_vars, "w_kin_com", w_kin_com_);
	    readParameter(planner_vars, "w_kin_lmom", w_kin_lmom_);
	    readParameter(planner_vars, "w_kin_amom", w_kin_amom_);
	    readParameter(planner_vars, "w_kin_lmomd", w_kin_lmomd_);
	    readParameter(planner_vars, "w_kin_amomd", w_kin_amomd_);
	    readParameter(planner_vars, "w_kin_eff_pos", w_kin_eff_pos_);
	    readParameter(planner_vars, "w_kin_base_ori", w_kin_base_ori_);
	    readParameter(planner_vars, "w_kin_joint_vel", w_kin_joint_vel_);
	    readParameter(planner_vars, "w_kin_joint_acc", w_kin_joint_acc_);
        readParameter(planner_vars, "slacks_penalty", kin_slacks_penalty_);
	    readParameter(planner_vars, "integration_step", kin_integration_step_);
	    readParameter(planner_vars, "w_kin_eff_pos_nonact", w_kin_eff_pos_nonact_);
	    readParameter(planner_vars, "w_kin_default_joints", w_kin_default_joints_);
	    readParameter(planner_vars, "max_trajectory_iters", max_trajectory_iters_);
	    readParameter(planner_vars, "max_convergence_iters", max_convergence_iters_);
	    readParameter(planner_vars, "convergence_tolerance", convergence_tolerance_);
        readParameter(planner_vars, "lambda_regularization", lambda_regularization_);
      }

      //kinematic momentum optimization
      readParameter(planner_vars, "swing_traj_via_z", swing_traj_via_z_);
      readParameter(planner_vars, "w_lin_mom_tracking", w_lin_mom_tracking_);
      readParameter(planner_vars, "w_ang_mom_tracking", w_ang_mom_tracking_);
      readParameter(planner_vars, "w_endeff_contact", w_endeff_contact_);
      readParameter(planner_vars, "w_endeff_tracking", w_endeff_tracking_);
      readParameter(planner_vars, "p_endeff_tracking", p_endeff_tracking_);
      readParameter(planner_vars, "p_com_tracking", p_com_tracking_);
      readParameter(planner_vars, "reg_orientation", reg_orientation_);

      // Storage information
      readParameter(planner_vars, "store_data", store_data_);

      // Solver setting
      readParameter(planner_vars, "use_default_solver_setting", use_default_solver_setting_);

      num_act_dofs_ = active_dofs_.size();
      mass_times_gravity_ = robot_mass_ * gravity_;
      gravity_vector_ = Eigen::Vector3d(0., 0., -gravity_);
      num_timesteps_ = std::floor(time_horizon_/time_step_);
      num_extended_act_dofs_ = extended_active_dofs_.size();
    }
    catch (std::runtime_error& e)
    {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
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

      // Kinematics momentum optimization weights
      case PlannerDoubleParam_SwingTrajViaZ : { return swing_traj_via_z_; }
      case PlannerDoubleParam_WeightLinMomentumTracking : { return w_lin_mom_tracking_; }
      case PlannerDoubleParam_WeightAngMomentumTracking : { return w_ang_mom_tracking_; }
      case PlannerDoubleParam_WeightEndEffContact : { return w_endeff_contact_; }
      case PlannerDoubleParam_WeightEndEffTracking : { return w_endeff_tracking_; }
      case PlannerDoubleParam_PGainEndEffTracking : { return p_endeff_tracking_; }
      case PlannerDoubleParam_PGainComTracking : { return p_com_tracking_; }
      case PlannerDoubleParam_PGainOrientationTracking : { return reg_orientation_; }

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

      // Kinematics momentum optimization weights
      case PlannerDoubleParam_SwingTrajViaZ : { return swing_traj_via_z_; }
      case PlannerDoubleParam_WeightLinMomentumTracking : { return w_lin_mom_tracking_; }
      case PlannerDoubleParam_WeightAngMomentumTracking : { return w_ang_mom_tracking_; }
      case PlannerDoubleParam_WeightEndEffContact : { return w_endeff_contact_; }
      case PlannerDoubleParam_WeightEndEffTracking : { return w_endeff_tracking_; }
      case PlannerDoubleParam_PGainEndEffTracking : { return p_endeff_tracking_; }
      case PlannerDoubleParam_PGainComTracking : { return p_com_tracking_; }
      case PlannerDoubleParam_PGainOrientationTracking : { return reg_orientation_; }

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

    // Not handled parameters
    default: { throw std::runtime_error("PlannerSetting::get PlannerCVectorParam invalid"); break; }
	}
  }

}
