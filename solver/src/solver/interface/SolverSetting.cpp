/**
 * @file SolverSetting.cpp
 * @author A. Domahidi [domahidi@embotech.com]
 * @license (new license) License BSD-3-Clause
 * @copyright Copyright (c) [2012-2015] Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 * @date 2012-2015
 * @brief ECOS - Embedded Conic Solver
 * 
 * Modified to c++ code by New York University and Max Planck Gesellschaft, 2017 
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  void SolverSetting::initialize(const std::string cfg_file, const std::string solver_vars_yaml)
  {
    try
    {
	  YAML::Node solver_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node solver_vars = solver_cfg[solver_vars_yaml.c_str()];

	  // Branch and Bound solver
	  BnB_verbose_ = solver_vars["BnB_verbose"].as<bool>();
	  BnB_max_iterations_ = solver_vars["BnB_max_iterations"].as<int>();
	  BnB_integer_tolerance_ = solver_vars["BnB_integer_tolerance"].as<double>();
	  BnB_absolute_suboptimality_gap_ = solver_vars["BnB_absolute_suboptimality_gap"].as<double>();
	  BnB_relative_suboptimality_gap_ = solver_vars["BnB_relative_suboptimality_gap"].as<double>();

	  // Convergence tolerances
	  feasibility_tolerance_ = solver_vars["feasibility_tolerance"].as<double>();
	  absolute_suboptimality_gap_ = solver_vars["absolute_suboptimality_gap"].as<double>();
	  relative_suboptimality_gap_ = solver_vars["relative_suboptimality_gap"].as<double>();
	  feasibility_tolerance_inaccurate_ = solver_vars["feasibility_tolerance_inaccurate"].as<double>();
	  absolute_suboptimality_gap_inaccurate_ = solver_vars["absolute_suboptimality_gap_inaccurate"].as<double>();
	  relative_suboptimality_gap_inaccurate_ = solver_vars["relative_suboptimality_gap_inaccurate"].as<double>();

	  // Equilibration parameters
	  equil_iterations_ = solver_vars["equil_iterations"].as<int>();

	  // Linear System parameters
	  dyn_reg_thresh_ = solver_vars["dyn_reg_thresh"].as<double>();
	  lin_sys_accuracy_ = solver_vars["lin_sys_accuracy"].as<double>();
	  err_reduction_factor_ = solver_vars["err_reduction_factor"].as<double>();
	  num_iter_ref_lin_solve_ = solver_vars["num_iter_ref_lin_solve"].as<int>();
	  static_regularization_ = solver_vars["static_regularization"].as<double>();
	  dynamic_regularization_ = solver_vars["dynamic_regularization"].as<double>();

      // Algorithm parameters
	  safeguard_ = solver_vars["safeguard"].as<double>();
	  min_step_length_ = solver_vars["min_step_length"].as<double>();
	  max_step_length_ = solver_vars["max_step_length"].as<double>();
	  min_centering_step_ = solver_vars["min_centering_step"].as<double>();
	  max_centering_step_ = solver_vars["max_centering_step"].as<double>();
	  step_length_scaling_ = solver_vars["step_length_scaling"].as<double>();

	  // Model parameters
	  verbose_ = solver_vars["verbose"].as<bool>();

	  max_iters_ = solver_vars["max_iters"].as<int>();
	  ipsolver_max_iters_ = solver_vars["ipsolver_max_iters"].as<int>();
	  ipsolver_warm_iters_ = solver_vars["ipsolver_warm_iters"].as<int>();

	  num_itrefs_trustregion_ = solver_vars["num_itrefs_trustregion"].as<int>();
	  trust_region_threshold_ = solver_vars["trust_region_threshold"].as<double>();
	  soft_constraint_weight_full_ = solver_vars["soft_constraint_weight_full"].as<double>();
	  soft_constraint_weight_reduced_ = solver_vars["soft_constraint_weight_reduced"].as<double>();
    }
    catch (YAML::ParserException &e)
    {
      std::cout << e.what() << "\n";
    }
  }

  // getter and setter methods for boolean parameters
  bool SolverSetting::get(SolverBoolParam param) const
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverBoolParam_BnBVerbose : { return BnB_verbose_; }

      // Model parameters
      case SolverBoolParam_Verbose: { return verbose_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::get SolverBoolParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverBoolParam param, bool value)
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverBoolParam_BnBVerbose : { BnB_verbose_ = value; break; }

      // Model parameters
      case SolverBoolParam_Verbose: { verbose_ = value; break; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::set SolverBoolParam invalid"); break; }
    }
  }

  // getter and setter methods for integer parameters
  int SolverSetting::get(SolverIntParam param) const
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverIntParam_BnBMaxIters : { return BnB_max_iterations_; }

      // Equilibration parameters
      case SolverIntParam_EquilibrationIters : { return equil_iterations_; }

      // Linear System parameters
      case SolverIntParam_NumIterRefinementsLinSolve : { return num_iter_ref_lin_solve_; }

      // Model parameters
      case SolverIntParam_MaxIters: { return max_iters_; }
      case SolverIntParam_SolverMaxIters : { return ipsolver_max_iters_; }
      case SolverIntParam_WarmStartIters : { return ipsolver_warm_iters_; }
      case SolverIntParam_NumberRefinementsTrustRegion : { return num_itrefs_trustregion_; }
      default: { throw std::runtime_error("SolverSetting::get SolverIntParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverIntParam param, int value)
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverIntParam_BnBMaxIters : { BnB_max_iterations_ = value; break; }

      // Equilibration parameters
      case SolverIntParam_EquilibrationIters : { equil_iterations_ = value; break; }

      // Linear System parameters
      case SolverIntParam_NumIterRefinementsLinSolve : { num_iter_ref_lin_solve_ = value; break; }

      // Model parameters
      case SolverIntParam_MaxIters : { max_iters_ = value; break; }
      case SolverIntParam_SolverMaxIters : { ipsolver_max_iters_ = value; break; }
      case SolverIntParam_WarmStartIters : { ipsolver_warm_iters_ = value; break; }
      case SolverIntParam_NumberRefinementsTrustRegion : { num_itrefs_trustregion_ = value; break; }
      default: { throw std::runtime_error("SolverSetting::set SolverIntParam invalid"); break; }
    }
  }

  // getter and setter methods for double parameters
  double SolverSetting::get(SolverDoubleParam param) const
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverDoubleParam_BnBIntegerTol : { return BnB_integer_tolerance_; }
      case SolverDoubleParam_BnBAbsSubOptGap : { return BnB_absolute_suboptimality_gap_; }
      case SolverDoubleParam_BnBRelSubOptGap : { return BnB_relative_suboptimality_gap_; }

      // Convergence tolerances
      case SolverDoubleParam_FeasibilityTol: { return feasibility_tolerance_; }
      case SolverDoubleParam_DualityGapAbsTol: { return absolute_suboptimality_gap_; }
      case SolverDoubleParam_DualityGapRelTol: { return relative_suboptimality_gap_; }
      case SolverDoubleParam_FeasibilityTolInacc: { return feasibility_tolerance_inaccurate_; }
      case SolverDoubleParam_DualityGapAbsTolInacc: { return absolute_suboptimality_gap_inaccurate_; }
      case SolverDoubleParam_DualityGapRelTolInacc: { return relative_suboptimality_gap_inaccurate_; }

      // Linear System parameters
      case SolverDoubleParam_LinearSystemAccuracy : { return lin_sys_accuracy_; }
      case SolverDoubleParam_ErrorReductionFactor : { return err_reduction_factor_; }
      case SolverDoubleParam_DynamicRegularizationThresh : { return dyn_reg_thresh_; }
      case SolverDoubleParam_StaticRegularization : { return static_regularization_; }
      case SolverDoubleParam_DynamicRegularization : { return dynamic_regularization_; }

      // Algorithm parameters
      case SolverDoubleParam_SafeGuard : { return safeguard_; }
      case SolverDoubleParam_MinimumStepLength : { return min_step_length_; }
      case SolverDoubleParam_MaximumStepLength : { return max_step_length_; }
      case SolverDoubleParam_StepLengthScaling : { return step_length_scaling_; }
      case SolverDoubleParam_MinimumCenteringStep : { return min_centering_step_; }
      case SolverDoubleParam_MaximumCenteringStep : { return max_centering_step_; }

      // Model parameters
      case SolverDoubleParam_TrustRegionThreshold : { return trust_region_threshold_; }
      case SolverDoubleParam_SoftConstraintWeightFull : { return soft_constraint_weight_full_; }
      case SolverDoubleParam_SoftConstraintWeightReduced : { return soft_constraint_weight_reduced_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::get SolverDoubleParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverDoubleParam param, double value)
  {
    switch (param)
    {
      // Branch and Bound solver
      case SolverDoubleParam_BnBIntegerTol : { BnB_integer_tolerance_ = value; break; }
      case SolverDoubleParam_BnBAbsSubOptGap : { BnB_absolute_suboptimality_gap_ = value; break; }
      case SolverDoubleParam_BnBRelSubOptGap : { BnB_relative_suboptimality_gap_ = value; break; }

      // Convergence tolerances
      case SolverDoubleParam_FeasibilityTol: { feasibility_tolerance_ = value; break; }
      case SolverDoubleParam_DualityGapAbsTol: { absolute_suboptimality_gap_ = value; break; }
      case SolverDoubleParam_DualityGapRelTol: { relative_suboptimality_gap_ = value; break; }
      case SolverDoubleParam_FeasibilityTolInacc: { feasibility_tolerance_inaccurate_ = value; break; }
      case SolverDoubleParam_DualityGapAbsTolInacc: { absolute_suboptimality_gap_inaccurate_ = value; break; }
      case SolverDoubleParam_DualityGapRelTolInacc: { relative_suboptimality_gap_inaccurate_ = value; break; }

      // Linear System parameters
      case SolverDoubleParam_LinearSystemAccuracy : { lin_sys_accuracy_ = value; break; }
      case SolverDoubleParam_ErrorReductionFactor : { err_reduction_factor_ = value; break; }
      case SolverDoubleParam_DynamicRegularizationThresh : { dyn_reg_thresh_ = value; break; }
      case SolverDoubleParam_StaticRegularization : { static_regularization_ = value; break; }
      case SolverDoubleParam_DynamicRegularization : { dynamic_regularization_ = value; break; }

      // Algorithm parameters
      case SolverDoubleParam_SafeGuard : { safeguard_ = value; break; }
      case SolverDoubleParam_MinimumStepLength : { min_step_length_ = value; break; }
      case SolverDoubleParam_MaximumStepLength : { max_step_length_ = value; break; }
      case SolverDoubleParam_StepLengthScaling : { step_length_scaling_ = value; break; }
      case SolverDoubleParam_MinimumCenteringStep : { min_centering_step_ = value; break; }
      case SolverDoubleParam_MaximumCenteringStep : { max_centering_step_ = value; break; }

      // Model parameters
      case SolverDoubleParam_TrustRegionThreshold : { trust_region_threshold_ = value; break; }
      case SolverDoubleParam_SoftConstraintWeightFull : { soft_constraint_weight_full_ = value; break; }
      case SolverDoubleParam_SoftConstraintWeightReduced : { soft_constraint_weight_reduced_ = value; break; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::set SolverDoubleParam invalid"); break; }
    }
  }

}
