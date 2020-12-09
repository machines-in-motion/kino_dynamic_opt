/**
 * @file SolverLqrSetting.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <iostream>
#include <yaml_utils/yaml_eigen.hpp>
#include <solver_lqr/SolverLqrSetting.hpp>

namespace solverlqr {

  void SolverLqrSetting::initialize(const std::string cfg_file, const std::string stgs_vars_yaml)
  {
	cfg_file_ = cfg_file;
	save_lqr_file_ = cfg_file.substr(0, cfg_file.size()-5) + "_lqr_results.yaml";

    try
    {
	  YAML::Node stgs_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node stgs_vars = stgs_cfg[stgs_vars_yaml.c_str()];

	  // LQR-like algorithm parameters
	  use_runge_kutta_integration_ = false;
      verbosity_ = stgs_vars["verbosity"].as<int>();
      lqr_max_iters_ = stgs_vars["lqr_max_iters"].as<int>();
      precision_digits_ = stgs_vars["decimal_digits"].as<int>();
      bpass_regularization_type_ = stgs_vars["bpass_regularization_type"].as<int>();

      cost_change_tolerance_ = stgs_vars["cost_change_tolerance"].as<double>();
      divergence_limit_check_ = stgs_vars["divergence_limit_check"].as<double>();
      bpass_min_regularization_ = stgs_vars["bpass_min_regularization"].as<double>();
      bpass_max_regularization_ = stgs_vars["bpass_max_regularization"].as<double>();
      control_gradient_tolerance_ = stgs_vars["control_gradient_tolerance"].as<double>();
      bpass_initial_regularization_ = stgs_vars["bpass_initial_regularization"].as<double>();
      min_expected_cost_improvement_ = stgs_vars["min_expected_cost_improvement"].as<double>();
      bpass_mult_regularization_incr_ = stgs_vars["bpass_mult_regularization_incr"].as<double>();
      bpass_initial_mult_regularization_incr_ = stgs_vars["bpass_initial_mult_regularization_incr"].as<double>();

      double linesearch_coeff = stgs_vars["linesearch_coeff"].as<double>();
      double linesearch_num_coeffs = stgs_vars["linesearch_num_coeffs"].as<double>();
      alpha_.resize(linesearch_num_coeffs);
      for (int id=0; id<linesearch_num_coeffs; id++)
        alpha_[id] = std::pow(linesearch_coeff, -(id*id));

      store_data_ = stgs_vars["store_data"].as<bool>();

      // LQR problem parameters
      time_step_ = stgs_vars["time_step"].as<double>();
      time_horizon_ = stgs_vars["time_horizon"].as<double>();
      state_dimension_ = stgs_vars["state_dimension"].as<int>();
      control_dimension_ = stgs_vars["control_dimension"].as<int>();
      initial_state_ = stgs_vars["initial_state"].as<Eigen::VectorXd>();
      time_dimension_ = std::floor(time_horizon_/time_step_);

      has_control_limits_ = stgs_vars["has_control_limits"].as<bool>();
      if (has_control_limits_) {
        min_control_limits_ = stgs_vars["min_control_limits"].as<Eigen::VectorXd>();
        max_control_limits_ = stgs_vars["max_control_limits"].as<Eigen::VectorXd>();
      }

      user_parameters_ = stgs_vars["user_parameters"];
    }
    catch (YAML::ParserException &e)
    {
      std::cout << e.what() << "\n";
    }
  }

  // getter and setter methods for boolean parameters
  bool& SolverLqrSetting::get(SolverLqrBoolParam param)
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrBoolParam_StoreData : { return store_data_; }
      case SolverLqrBoolParam_HasControlLimits : { return has_control_limits_; }
      case SolverLqrBoolParam_UseRungeKuttaIntegration : { return use_runge_kutta_integration_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get BoolParam invalid"); break; }
    }
  }

  const bool& SolverLqrSetting::get(SolverLqrBoolParam param) const
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrBoolParam_StoreData : { return store_data_; }
      case SolverLqrBoolParam_HasControlLimits : { return has_control_limits_; }
      case SolverLqrBoolParam_UseRungeKuttaIntegration : { return use_runge_kutta_integration_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get BoolParam invalid"); break; }
    }
  }

  // getter and setter methods for integer parameters
  int& SolverLqrSetting::get(SolverLqrIntParam param)
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrIntParam_Verbosity: { return verbosity_; }
      case SolverLqrIntParam_LqrMaxIterations: { return lqr_max_iters_; }
      case SolverLqrIntParam_PrecisionDigits: { return precision_digits_; }
      case SolverLqrIntParam_BackPassRegulatizationType: { return bpass_regularization_type_; }

      // LQR problem parameters
      case SolverLqrIntParam_TimeDimension : { return time_dimension_; }
      case SolverLqrIntParam_StateDimension : { return state_dimension_; }
      case SolverLqrIntParam_ControlDimension : { return control_dimension_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get IntParam invalid"); break; }
    }
  }

  const int& SolverLqrSetting::get(SolverLqrIntParam param) const
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrIntParam_Verbosity: { return verbosity_; }
      case SolverLqrIntParam_LqrMaxIterations: { return lqr_max_iters_; }
      case SolverLqrIntParam_PrecisionDigits: { return precision_digits_; }
      case SolverLqrIntParam_BackPassRegulatizationType: { return bpass_regularization_type_; }

      // LQR problem parameters
      case SolverLqrIntParam_TimeDimension : { return time_dimension_; }
      case SolverLqrIntParam_StateDimension : { return state_dimension_; }
      case SolverLqrIntParam_ControlDimension : { return control_dimension_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get IntParam invalid"); break; }
    }
  }

  // getter and setter methods for double parameters
  double& SolverLqrSetting::get(SolverLqrDoubleParam param)
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrDoubleParam_CostChangeTolerance : { return cost_change_tolerance_; }
      case SolverLqrDoubleParam_DivergenceLimitCheck : { return divergence_limit_check_; }
      case SolverLqrDoubleParam_BackPassMinRegularization : { return bpass_min_regularization_; }
      case SolverLqrDoubleParam_BackPassMaxRegularization : { return bpass_max_regularization_; }
      case SolverLqrDoubleParam_ControlGradientTolerance : { return control_gradient_tolerance_; }
      case SolverLqrDoubleParam_MinExpectedCostImprovement : { return min_expected_cost_improvement_; }
      case SolverLqrDoubleParam_BackPassInitialRegularization : { return bpass_initial_regularization_; }
      case SolverLqrDoubleParam_BackPassMultRegularizationIncr : { return bpass_mult_regularization_incr_; }
      case SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr : { return bpass_initial_mult_regularization_incr_; }

      // LQR problem parameters
      case SolverLqrDoubleParam_TimeStep : { return time_step_; }
      case SolverLqrDoubleParam_TimeHorizon : { return time_horizon_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get DoubleParam invalid"); break; }
    }
  }

  const double& SolverLqrSetting::get(SolverLqrDoubleParam param) const
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrDoubleParam_CostChangeTolerance : { return cost_change_tolerance_; }
      case SolverLqrDoubleParam_DivergenceLimitCheck : { return divergence_limit_check_; }
      case SolverLqrDoubleParam_BackPassMinRegularization : { return bpass_min_regularization_; }
      case SolverLqrDoubleParam_BackPassMaxRegularization : { return bpass_max_regularization_; }
      case SolverLqrDoubleParam_ControlGradientTolerance : { return control_gradient_tolerance_; }
      case SolverLqrDoubleParam_MinExpectedCostImprovement : { return min_expected_cost_improvement_; }
      case SolverLqrDoubleParam_BackPassInitialRegularization : { return bpass_initial_regularization_; }
      case SolverLqrDoubleParam_BackPassMultRegularizationIncr : { return bpass_mult_regularization_incr_; }
      case SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr : { return bpass_initial_mult_regularization_incr_; }

      // LQR problem parameters
      case SolverLqrDoubleParam_TimeStep : { return time_step_; }
      case SolverLqrDoubleParam_TimeHorizon : { return time_horizon_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get DoubleParam invalid"); break; }
    }
  }

  // getter and setter methods for string parameters
  std::string& SolverLqrSetting::get(SolverLqrStringParam param)
  {
	switch (param)
	{
	  // Storage information
	  case SolverLqrStringParam_ConfigFile : { return cfg_file_; }
	  case SolverLqrStringParam_SaveLqrFile : { return save_lqr_file_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get StringParam invalid"); break; }
	}
  }

  const std::string& SolverLqrSetting::get(SolverLqrStringParam param) const
  {
	switch (param)
	{
	  // Storage information
	  case SolverLqrStringParam_ConfigFile : { return cfg_file_; }
	  case SolverLqrStringParam_SaveLqrFile : { return save_lqr_file_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get StringParam invalid"); break; }
	}
  }

  // getter and setter methods for vector parameters
  Eigen::VectorXd& SolverLqrSetting::get(SolverLqrVectorParam param)
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrVectorParam_LineSearchCoeffs : { return alpha_; }
      case SolverLqrVectorParam_InitialState : { return initial_state_; }
      case SolverLqrVectorParam_MinControlLimits : { return min_control_limits_; }
      case SolverLqrVectorParam_MaxControlLimits : { return max_control_limits_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get VectorParam invalid"); break; }
    }
  }

  const Eigen::VectorXd& SolverLqrSetting::get(SolverLqrVectorParam param) const
  {
    switch (param)
    {
      // LQR-like algorithm parameters
      case SolverLqrVectorParam_LineSearchCoeffs : { return alpha_; }
      case SolverLqrVectorParam_InitialState : { return initial_state_; }
      case SolverLqrVectorParam_MinControlLimits : { return min_control_limits_; }
      case SolverLqrVectorParam_MaxControlLimits : { return max_control_limits_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get VectorParam invalid"); break; }
    }
  }

  // getter and setter methods for yaml parameters
  YAML::Node& SolverLqrSetting::get(SolverLqrYamlParam param)
  {
    switch (param)
    {
      // LQR problem parameters
      case SolverLqrYamlParam_UserParameters : { return user_parameters_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get VectorParam invalid"); break; }
    }
  }

  const YAML::Node& SolverLqrSetting::get(SolverLqrYamlParam param) const
  {
    switch (param)
    {
      // LQR problem parameters
      case SolverLqrYamlParam_UserParameters : { return user_parameters_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverLqrSetting::get VectorParam invalid"); break; }
    }
  }

}
