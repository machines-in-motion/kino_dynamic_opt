/**
 * @file Clock.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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
