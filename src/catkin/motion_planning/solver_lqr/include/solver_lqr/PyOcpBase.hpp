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

#include <memory>
#include <Eigen/Eigen>

#include <solver_lqr/OcpDescription.hpp>

namespace solverlqr {

  class PyOcpBase : public OcpBase
  {
    public:
      using OcpBase::OcpBase;

      // trampoline for pure virtual functions
      void configure(const YAML::Node& user_parameters) override
      {
        PYBIND11_OVERLOAD_PURE(
          void,
		  OcpBase,
		  configure,
		  user_parameters
        );
      }

      StateBase dynamics(const StateBase& state, const ControlBase& control, int time_id) override
      {
        PYBIND11_OVERLOAD_PURE(
          StateBase,
		  OcpBase,
		  dynamics,
		  state,
          control,
		  time_id
        );
      }

      double objective(const StateBase& state, const ControlBase& control, int time_id, bool is_final_timestep) override
      {
        PYBIND11_OVERLOAD_PURE(
          double,
		  OcpBase,
		  objective,
		  state,
		  control,
		  time_id,
		  is_final_timestep
        );
      }
  };

}
