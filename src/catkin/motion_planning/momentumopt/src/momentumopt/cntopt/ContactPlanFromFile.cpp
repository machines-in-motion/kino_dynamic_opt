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
  void ContactPlanFromFile::optimize(const DynamicsState& ini_state, const TerrainDescription& terrain)
  {
    // this function fills in the contact sequence by reading it from a file
    this->contactSequence().loadFromFile(this->getSetting().get(PlannerStringParam_ConfigFile));
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
      for (int cnt_id=0; cnt_id<this->contactSequence().endeffectorContacts(eff_id).size(); cnt_id++)
        this->contactSequence().endeffectorContacts(eff_id)[cnt_id].selectedAsActive() = true;
  }

}
