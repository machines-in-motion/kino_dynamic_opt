/**
 * @file KinematicsInterface.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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
      virtual Eigen::Vector3d logarithmicMap(const Eigen::Vector4d quat_wxyz) = 0;
      virtual KinematicsState integratePosture(KinematicsState& kin_state, double dt) = 0;
      virtual void displayPosture(const KinematicsState& kin_state, double time_step) = 0;
      virtual KinematicsState updateJacobiansAndState(KinematicsState& kin_state, double dt) = 0;
      virtual KinematicsState differentiatePostures(KinematicsState& start_state, KinematicsState& end_state, double timestep) = 0;

      // center of mass and momentum jacobians
      Eigen::MatrixXd& centerOfMassJacobian() { return center_of_mass_jacobian_; }
      Eigen::MatrixXd& centroidalMomentumMatrix() { return centroidal_momentum_matrix_; }
      const Eigen::MatrixXd& centerOfMassJacobian() const { return center_of_mass_jacobian_; }
      const Eigen::MatrixXd& centroidalMomentumMatrix() const { return centroidal_momentum_matrix_; }
      void centerOfMassJacobian(const Eigen::MatrixXd& center_of_mass_jacobian) { center_of_mass_jacobian_ = center_of_mass_jacobian; }
      void centroidalMomentumMatrix(const Eigen::MatrixXd& centroidal_mometum_matrix) { centroidal_momentum_matrix_ = centroidal_mometum_matrix; }

      Eigen::MatrixXd& centroidalMomentumMatrixVariation() { return centroidal_momentum_matrix_variation_; }
      const Eigen::MatrixXd& centroidalMomentumMatrixVariation() const { return centroidal_momentum_matrix_variation_; }
      void centroidalMomentumMatrixVariation(const Eigen::MatrixXd& centroidal_momentum_matrix_variation) { centroidal_momentum_matrix_variation_ = centroidal_momentum_matrix_variation; }

      // endeffector jacobians
      Eigen::MatrixXd& endeffectorJacobian(int eff_id) { return endeffector_jacobians_[eff_id]; }
      const std::vector<Eigen::MatrixXd>& endeffectorJacobians() const { return endeffector_jacobians_; }
      const Eigen::MatrixXd& endeffectorJacobian(int eff_id) const { return endeffector_jacobians_[eff_id]; }
      void endeffectorJacobians(const std::vector<Eigen::MatrixXd>& endeffector_jacobians) { endeffector_jacobians_ = endeffector_jacobians; }

      // bodies non-penetration constraints
      const Eigen::VectorXd& constraintsVector() const { return constraints_vector_; }
      const Eigen::MatrixXd& constraintsMatrix() const { return constraints_matrix_; }
      void constraintsVector(const Eigen::VectorXd& constraints_vector) { constraints_vector_ = constraints_vector; }
      void constraintsMatrix(const Eigen::MatrixXd& constraints_matrix) { constraints_matrix_ = constraints_matrix; }

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
      Eigen::VectorXd constraints_vector_;
      std::vector<Eigen::MatrixXd> endeffector_jacobians_;
      Eigen::MatrixXd centroidal_momentum_matrix_, centroidal_momentum_matrix_variation_, center_of_mass_jacobian_, constraints_matrix_;
  };

}
