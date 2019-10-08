/**
 * @file KinematicsInterface.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/kinopt/KinematicsInterface.hpp>

namespace momentumopt {

  // KinematicsInterface class functions implementation
  void KinematicsInterface::internalInitialization(PlannerSetting& planner_setting)
  {
    planner_setting_ = &planner_setting;

    center_of_mass_jacobian_.resize(3, 6+this->getSetting().get(PlannerIntParam_NumDofs));
    centroidal_momentum_matrix_.resize(6, 6+this->getSetting().get(PlannerIntParam_NumDofs));
    centroidal_momentum_matrix_variation_.resize(6, 6+this->getSetting().get(PlannerIntParam_NumDofs));

    endeffector_jacobians_.clear();
    center_of_mass_jacobian_.setZero();
    centroidal_momentum_matrix_.setZero();
    centroidal_momentum_matrix_variation_.setZero();
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
      endeffector_jacobians_.push_back(Eigen::MatrixXd(6, 6+this->getSetting().get(PlannerIntParam_NumDofs)).setZero());

    this->initialize(planner_setting);
  }

}
