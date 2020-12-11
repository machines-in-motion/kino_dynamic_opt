/**
 * @file RtHQPCost.hpp
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <cassert>
#include <iostream>

namespace rt_solver {

  class RtHQPCost
  {
    public:
      RtHQPCost(){};
      virtual ~RtHQPCost(){};

      virtual int maxRank() const = 0;
      virtual void updateAfterSolutionFound() = 0;
      virtual void addCostToHierarchy(int rank) const = 0;
      virtual void addCostToHierarchyAfterReduction(int /*rank*/) const{};
  };
}
