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

#include <vector>
#include <memory>
#include <Eigen/Eigen>

#include <momentumopt/setting/PlannerSetting.hpp>
//#include <solver/interface/NlpDescription.hpp>
//#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/kinopt/KinematicsState.hpp>

namespace momentumopt {

  class KinematicsOptimizer;

  class KinematicsInterface// : public virtual solver::NlpDescription
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
	  KinematicsInterface(){}
      ~KinematicsInterface(){}

      // pure virtual functions to be implemented
      virtual void initialize(PlannerSetting& planner_setting) = 0;
      virtual KinematicsState updateJacobians(const KinematicsState& kin_state) = 0;


//      // pure virtual functions to be implemented
//      virtual void displayPosture(const DynamicsState& state, double time_step) = 0;
//      virtual double endeffectorOrientationError(int n_vars, const double* x) = 0;
//      virtual void updateJacobianAndState(Eigen::Ref<Eigen::MatrixXd> centroidal_momentum_matrix, Eigen::Ref<Eigen::MatrixXd> base_jacobian,
//                          std::array<Eigen::MatrixXd, Problem::n_endeffs_>& endeffector_jacobian, DynamicsState& current_joints_state) = 0;
//      virtual void updateInertiaAndNonlinearTerms(Eigen::Ref<Eigen::MatrixXd> inertia_matrix,
//                          Eigen::Ref<Eigen::VectorXd> nonlinear_terms, const DynamicsState& current_joints_state) = 0;
//
//      // helper functions to deal with joints in charge of end-effector rotations
//      const Eigen::VectorXi& endeffectorRotationalDofs() const { return eff_rotation_dofs_; }
//      void initialize(const Eigen::VectorXi& eff_rotation_dofs) { eff_rotation_dofs_ = eff_rotation_dofs; }
//      void setDesiredEndeffectorOrientation(const DynamicsState& state) { des_eff_orientation_ = std::make_shared<DynamicsState>(state); }
//      const Eigen::Quaternion<double>& getDesiredEndeffectorOrientation(int eff_id) const { return des_eff_orientation_->endeffectorOrientation(eff_id); }
//
//      // helper functions to define how to optimize endeffector-rotations
//      void getNlpParameters(int& n_vars, int& n_cons) { n_vars = 12; n_cons = 0; }
//      void getStartingPoint(int n_vars, double* x) { for (int i=0; i<n_vars; i++) { x[i]=0.0; } }
//      void evaluateConstraintsVector(int n_vars, int n_cons, const double* x, double* constraints) {}
//      double evaluateObjective(int n_vars, const double* x) { return this->endeffectorOrientationError(n_vars, x); }
//      void getNlpBounds(int n_vars, int n_cons, double* x_l, double* x_u, double* g_l, double* g_u) { for (int i=0; i<n_vars; i++) { x_l[i] = -2*M_PI; x_u[i] =  2*M_PI; } }
//
//    private:
//      Eigen::VectorXi eff_rotation_dofs_;
//      std::shared_ptr<DynamicsState> des_eff_orientation_;

      Eigen::MatrixXd& centroidalMomentumMatrix() { return centroidal_mometum_matrix_; }
      const Eigen::MatrixXd& centroidalMomentumMatrix() const { return centroidal_mometum_matrix_; }
      void centroidalMomentumMatrix(const Eigen::MatrixXd& centroidal_mometum_matrix) { centroidal_mometum_matrix_ = centroidal_mometum_matrix; }

      Eigen::MatrixXd& endeffectorJacobian(int eff_id) { return endeffector_jacobians_[eff_id]; }
      const Eigen::MatrixXd& endeffectorJacobian(int eff_id) const { return endeffector_jacobians_[eff_id]; }

      const std::vector<Eigen::MatrixXd>& endeffectorJacobians() const { return endeffector_jacobians_; }
      void endeffectorJacobians(const std::vector<Eigen::MatrixXd>& endeffector_jacobians) { endeffector_jacobians_ = endeffector_jacobians; }

    private:
      friend class KinematicsOptimizer;
      /*! internal initialization function */
      void internalInitialization(PlannerSetting& planner_setting);

      /*! Getter and setter methods for getting the planner variables  */
      inline PlannerSetting& getSetting() { return *planner_setting_; }
      inline const PlannerSetting& getSetting() const { return *planner_setting_; }

    private:
      /*! Variables required for optimization */
      PlannerSetting* planner_setting_;

      /*! Jacobians and problem matrices */
      Eigen::MatrixXd centroidal_mometum_matrix_;
      std::vector<Eigen::MatrixXd> endeffector_jacobians_;
  };

}
