/**
 * @file ContactPlanFromFile.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <momentumopt/cntopt/ContactPlanInterface.hpp>

namespace momentumopt {

  /**
   * This class implements the ContactPlanInterface by reading a sequence of contacts
   * from a configuration file.
   */
  class ContactPlanFromFile : public ContactPlanInterface
  {
    public:
      ContactPlanFromFile(){}
      ~ContactPlanFromFile(){}

      virtual void optimize(const DynamicsState& ini_state, const TerrainDescription& terrain);
  };
}
