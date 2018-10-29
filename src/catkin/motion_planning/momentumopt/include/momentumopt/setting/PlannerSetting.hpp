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

#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <solver/interface/SolverSetting.hpp>
#include <momentumopt/cntopt/ContactState.hpp>
#include <momentumopt/setting/Definitions.hpp>
#include <momentumopt/setting/PlannerParams.hpp>

namespace momentumopt {

  enum class Heuristic { TrustRegion, SoftConstraint, TimeOptimization };

  class PlannerSetting
  {
    public:
	  PlannerSetting() : num_dofs_(0) {}
	  ~PlannerSetting(){}

	  void initialize(const std::string& cfg_file, const std::string& planner_vars_yaml = "planner_variables");

      int& get(PlannerIntParam param);
      bool& get(PlannerBoolParam param);
      double& get(PlannerDoubleParam param);
      std::string& get(PlannerStringParam param);
      Eigen::Ref<Eigen::VectorXd> get(PlannerVectorParam param);
      std::vector<Eigen::VectorXd>& get(PlannerCVectorParam param);
      Eigen::Ref<Eigen::VectorXi> get(PlannerIntVectorParam param);
      std::array<Eigen::VectorXd, Problem::n_endeffs_>& get(PlannerArrayParam param);

      const int& get(PlannerIntParam param) const;
      const bool& get(PlannerBoolParam param) const;
      const double& get(PlannerDoubleParam param) const;
      const std::string& get(PlannerStringParam param) const;
      const std::vector<Eigen::VectorXd>& get(PlannerCVectorParam param) const;
      const Eigen::Ref<const Eigen::VectorXd> get(PlannerVectorParam param) const;
      const Eigen::Ref<const Eigen::VectorXi> get(PlannerIntVectorParam param) const;
      const std::array<Eigen::VectorXd, Problem::n_endeffs_>& get(PlannerArrayParam param) const;

      Heuristic& heuristic() { return heuristic_; }
      const Heuristic& heuristic() const { return heuristic_; }

    private:
	  /*! Type of heuristic to use in the optimization problem */
	  Heuristic heuristic_;

	  /*! helper string variables for the optimization problem */
	  std::string cfg_file_, save_dynamics_file_, default_solver_setting_file_;

	  /*! helper integer variables for the optimization problem */
      int num_com_viapoints_, num_act_eefs_, num_timesteps_, max_time_iterations_, num_dofs_,
          num_act_dofs_, num_extended_act_dofs_, max_convergence_iters_;

      /*! helper boolean variables for the optimization problem */
      bool store_data_, is_time_horizon_fixed_, is_friction_cone_linear_, use_default_solver_setting_,
           load_kinematics_;

      /*! helper double variables for the optimization problem */
      double gravity_, time_step_, robot_mass_, time_horizon_, friction_coeff_, max_time_residual_tolerance_,
             min_time_residual_improvement_, mass_times_gravity_, w_trq_arm_, w_trq_leg_, w_time_penalty_, w_time_,
             convergence_tolerance_;

      /*! helper vector variables for the optimization problem */
      Eigen::Vector2d time_range_, torque_range_;
      Eigen::Vector3d external_force_, com_displacement_;
      Eigen::Vector3d w_com_, w_lmom_, w_lmomd_, w_lmom_final_, w_amom_, w_amomd_, w_amom_final_,
	                  w_com_via_, w_frc_arm_, w_frc_leg_, w_dfrc_arm_, w_dfrc_leg_, gravity_vector_,
                      w_lmom_track_, w_amom_track_,
                      w_kin_com_, w_kin_lmom_, w_kin_amom_, w_kin_lmomd_, w_kin_amomd_, w_kin_eff_pos_;
      Eigen::VectorXd default_joint_positions_, w_kin_default_joints_, w_kin_joint_vel_, w_kin_joint_acc_,
                      min_joint_limits_, max_joint_limits_;
      Eigen::VectorXi active_dofs_, extended_active_dofs_;

      /*! via points for center of mass motion */
      std::vector<Eigen::VectorXd> com_viapoints_;

      /*! region of support for end-effectors */
      std::array<Eigen::VectorXd, Problem::n_endeffs_> cop_range_;

      /*! offset of end-effectors from center of mass */
      std::array<Eigen::VectorXd, Problem::n_endeffs_> eff_offset_;

      /*! maximum end-effector lengths for legs and arms */
      Eigen::Matrix<double,Problem::n_endeffs_,1> max_eef_lengths_;
  };

}
