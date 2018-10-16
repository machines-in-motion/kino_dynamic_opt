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

#include <chrono>

namespace momentumopt {

  /**
   * Helper class to measure time required to run an optimization procedure.
   */
  class Clock {
    public:
      Clock(){}
      ~Clock(){}

    	  void start() { ini_time_ = std::chrono::high_resolution_clock::now(); }
    	  double stop() {
    	    end_time_ = std::chrono::high_resolution_clock::now();
    	    duration_ = std::chrono::duration_cast<std::chrono::microseconds>(end_time_-ini_time_);
    	    return double(duration_.count())*1e-3;
    	  }

    private:
      std::chrono::microseconds duration_;
      std::chrono::high_resolution_clock::time_point ini_time_, end_time_;
  };
}
