/*
 * Copyright [2019] Max Planck Society. All rights reserved.
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
      virtual void addCostToHierarchyAfterReduction(int rank) const{};
  };
}
