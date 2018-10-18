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

#include <momentumopt/kinopt/KinematicsInterface.hpp>

namespace momentumopt {

  // KinematicsInterface class functions implementation
  void KinematicsInterface::internalInitialization(PlannerSetting& planner_setting)
  {
    planner_setting_ = &planner_setting;

    centroidal_mometum_matrix_.resize(6, this->getSetting().get(PlannerIntParam_NumDofs));
    centroidal_mometum_matrix_.setZero();

    endeffector_jacobians_.clear();
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      endeffector_jacobians_.push_back(Eigen::MatrixXd(6, this->getSetting().get(PlannerIntParam_NumDofs)));
      endeffector_jacobians_[eff_id].setZero();
    }
    this->initialize(planner_setting);
  }

}
