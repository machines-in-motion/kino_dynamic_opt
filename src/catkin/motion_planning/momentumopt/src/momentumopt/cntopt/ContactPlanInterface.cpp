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

#include <momentumopt/cntopt/ContactPlanInterface.hpp>

namespace momentumopt {

  // ContactPlanInterface
  void ContactPlanInterface::initialize(const PlannerSetting& planner_setting)
  {
    planner_setting_ = &planner_setting;
  }

  void ContactPlanInterface::fillDynamicsSequence(DynamicsSequence& dynamics_sequence)
  {
	// configuration of end-effectors during contact
    dynamics_sequence.activeEndeffectorSteps().setZero();
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
      int counter = 0;
      int ini_id = 0, end_id = contact_sequence_.endeffectorContacts(eff_id).size();
      for (int cnt_id=ini_id; cnt_id<end_id; cnt_id++) {
        if (contact_sequence_.endeffectorContacts(eff_id)[cnt_id].selectedAsActive()) {
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
            double current_time = double(time_id+1.0)*this->getSetting().get(PlannerDoubleParam_TimeStep);
            if (current_time>=contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactActivationTime() &&
                current_time< contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime())
            {
              dynamics_sequence.dynamicsState(time_id).endeffectorActivation(eff_id) = true;
              dynamics_sequence.dynamicsState(time_id).endeffectorContactId(eff_id) = cnt_id;
              dynamics_sequence.dynamicsState(time_id).endeffectorActivationId(eff_id) = counter++;
            }
          }
        }
      }
      dynamics_sequence.activeEndeffectorSteps()[eff_id] = counter;
    }
  }

}
