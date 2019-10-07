/**
 * @file SolverLqrSetting.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#pragma once

#include <string>
#include <limits>
#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/SolverLqrParams.hpp>

namespace solverlqr {

  class SolverLqrSetting
  {
    public:
	  SolverLqrSetting(){}
	  ~SolverLqrSetting(){}

	  void initialize(const std::string cfg_file, const std::string stgs_vars_yaml = "solverlqr_variables");

      int& get(SolverLqrIntParam param);
      bool& get(SolverLqrBoolParam param);
      double& get(SolverLqrDoubleParam param);
      YAML::Node& get(SolverLqrYamlParam param);
      std::string& get(SolverLqrStringParam param);
      Eigen::VectorXd& get(SolverLqrVectorParam param);

      const int& get(SolverLqrIntParam param) const;
      const bool& get(SolverLqrBoolParam param) const;
      const double& get(SolverLqrDoubleParam param) const;
      const YAML::Node& get(SolverLqrYamlParam param) const;
      const std::string& get(SolverLqrStringParam param) const;
      const Eigen::VectorXd& get(SolverLqrVectorParam param) const;

	  static constexpr double nan = ((double)0x7ff8000000000000);
	  static constexpr double inf = ((double)std::numeric_limits<double>::infinity());

    private:
	  /*! helper yaml variables for the solver LQR */
	  YAML::Node user_parameters_;

	  /*! helper vector variables for the solver LQR */
	  Eigen::VectorXd alpha_, initial_state_, min_control_limits_, max_control_limits_;

	  /*! helper string variables for the solver LQR */
	  std::string cfg_file_, save_lqr_file_;

	  /*! helper boolean variables for the solver LQR */
	  bool store_data_, has_control_limits_, use_runge_kutta_integration_;

	  /*! helper integer variables for the solver LQR */
	  int verbosity_, lqr_max_iters_, precision_digits_, bpass_regularization_type_,
	      time_dimension_, state_dimension_, control_dimension_;

	  /*! helper double variables for the solver LQR */
	  double cost_change_tolerance_, divergence_limit_check_, bpass_min_regularization_,
             bpass_max_regularization_, control_gradient_tolerance_, bpass_initial_regularization_,
             time_step_, time_horizon_, min_expected_cost_improvement_, bpass_mult_regularization_incr_,
             bpass_initial_mult_regularization_incr_;
  };

}
