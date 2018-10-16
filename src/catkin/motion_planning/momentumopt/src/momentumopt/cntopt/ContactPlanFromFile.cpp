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

#include <momentumopt/cntopt/ContactPlanFromFile.hpp>

namespace momentumopt {

  // ContactPlan
  void ContactPlanFromFile::optimize(const DynamicsState& ini_state)
  {
    // this function fills in the contact sequence by reading it from a file
    try
    {
      YAML::Node planner_cfg = YAML::LoadFile(this->getSetting().get(PlannerStringParam_ConfigFile));
      YAML::Node contact_vars = planner_cfg["contact_plan"];

      // Contact parameters
      readParameter(contact_vars, "num_contacts", this->endeffectorContacts());
      for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
        this->contactSequence().endeffectorContacts(eff_id).clear();
        if (this->endeffectorContacts(eff_id)>0) {
          YAML::Node eff_params = contact_vars[("effcnt_" + Problem::idToEndeffectorString(eff_id)).c_str()];
          for (int cnt_id=0; cnt_id<this->endeffectorContacts(eff_id); cnt_id++) {
      	    Eigen::VectorXd v(10);
      	    readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
      	    this->contactSequence().endeffectorContacts(eff_id).push_back(ContactState());
      	    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactActivationTime() = v[0];
      	    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime() = v[1];
      	    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactPosition() = v.segment<3>(2);
      	    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactType() = idToContactType(v(9));
      	    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation() = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
          }
        }
      }
    }
    catch (std::runtime_error& e)
    {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
  }

}
