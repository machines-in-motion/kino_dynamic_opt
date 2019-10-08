/**
 * @file ContactPlanFromFile.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <momentumopt/cntopt/ContactPlanFromFile.hpp>

namespace momentumopt {

  // ContactPlan
  void ContactPlanFromFile::optimize(const DynamicsState& ini_state, const TerrainDescription& terrain)
  {
    // this function fills in the contact sequence by reading it from a file
    this->contactSequence().loadFromFile(this->getSetting().get(PlannerStringParam_ConfigFile));
    for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++)
      for (int cnt_id=0; cnt_id<(int)this->contactSequence().endeffectorContacts(eff_id).size(); cnt_id++)
        this->contactSequence().endeffectorContacts(eff_id)[cnt_id].selectedAsActive() = true;

    if (this->getSetting().get(PlannerBoolParam_LoadKinematics))
      this->viapointSequence().loadFromFile(this->getSetting().get(PlannerStringParam_ConfigFile));
  }

}
