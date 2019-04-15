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

#include <yaml_cpp_catkin/yaml_eigen.h>

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

  /*! helper function to safely read a yaml parameter */
  template <typename YamlType>
  static YamlType readParameter(const YAML::Node& node, const std::string& name) {
    try { return node[name.c_str()].as<YamlType>(); }
    catch (...) { throw std::runtime_error(name); }
  }

  template <typename YamlType>
  void readParameter(const YAML::Node& node, const std::string& name, YamlType& parameter) {
    try { parameter = readParameter<YamlType>(node, name); }
    catch (...) { throw std::runtime_error(name); }
  }

}
