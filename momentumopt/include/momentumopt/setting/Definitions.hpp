/**
 * @file Definitions.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#pragma once

#include <yaml_utils/yaml_cpp_fwd.hpp>

namespace momentumopt {

  /**
   * Helper structure used to define common functionality and naming conventions
   */
  struct Problem
  {
    /*! number of maximum end-effectors */
    static constexpr int n_endeffs_ = 4;

    /*! definition of end-effector id's */
    enum class EffId { id_right_foot = 0,
                       id_left_foot  = 1,
                       id_right_hand = 2,
                       id_left_hand  = 3 };

    /*! function to differentiate between hands and feet */
    static bool isHand(int eff_id) {
      if (eff_id == static_cast<int>(EffId::id_left_hand) ||
          eff_id == static_cast<int>(EffId::id_right_hand))
        return true;
      return false;
    }

    /*! function to map and integer eff_id to string eff_id */
    static std::string idToEndeffectorString(int eff_id) {
    	  switch (eff_id) {
    	    case static_cast<int>(EffId::id_right_foot): { return std::string("rf"); break; }
    	    case static_cast<int>(EffId::id_left_foot ): { return std::string("lf"); break; }
    	    case static_cast<int>(EffId::id_right_hand): { return std::string("rh"); break; }
    	    case static_cast<int>(EffId::id_left_hand ): { return std::string("lh"); break; }
    	    default: { throw std::runtime_error("eff_id not handled in idToEndeffectorString. \n"); }
    	  }
    }

  };
}
