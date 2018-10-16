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

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <momentumopt/cntopt/ContactState.hpp>
#include <momentumopt/dynopt/DynamicsState.hpp>
#include <momentumopt/setting/PlannerSetting.hpp>

namespace momentumopt {

  /**
   * This class implements a simple interface to store a sequence of
   * contacts, including its activation and de-activation time, position
   * and orientation of the contact, and a variable (set to 1) indicating
   * if the forces at this contact are subject to friction cone constraints
   * or (set to 2) if not, such as in the case of hands grasping bars.
   * It also fills the corresponding fields of DynamicsState within the
   * DynamicsSequence, namely contact positions, orientations, activations,
   * to be used by the dynamics optimizer.
   */
  class ContactPlanInterface
  {
    public:
      ContactPlanInterface(){}
      ~ContactPlanInterface(){}

      void initialize(const PlannerSetting& planner_setting);
      virtual void optimize(const DynamicsState& ini_state) = 0;
      void fillDynamicsSequence(DynamicsSequence& dynamics_sequence);

      ContactSequence& contactSequence() { return contact_sequence_; }
      const ContactSequence& contactSequence() const { return contact_sequence_; }
      int endeffectorContacts(int eff_id) const { return contacts_per_endeff_[eff_id]; }
      Eigen::Matrix<int, Problem::n_endeffs_, 1>& endeffectorContacts() { return contacts_per_endeff_; }
      const Eigen::Matrix<int, Problem::n_endeffs_, 1>& endeffectorContacts() const { return contacts_per_endeff_; }

    protected:
      /*! Getter and setter methods for getting the planner variables  */
      inline const PlannerSetting& getSetting() const { return *planner_setting_; }

    protected:
      ContactSequence contact_sequence_;
      const PlannerSetting* planner_setting_;
      Eigen::Matrix<int, Problem::n_endeffs_, 1> contacts_per_endeff_;
  };
}
