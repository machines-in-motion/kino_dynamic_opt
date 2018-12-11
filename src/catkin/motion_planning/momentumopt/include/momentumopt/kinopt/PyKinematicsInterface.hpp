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

#include <momentumopt/kinopt/KinematicsInterface.hpp>

namespace momentumopt {

  class PyKinematicsInterface : public KinematicsInterface
  {
    public:
      using KinematicsInterface::KinematicsInterface;

      // trampoline for pure virtual functions
      void initialize(PlannerSetting& planner_setting) override
      {
        PYBIND11_OVERLOAD_PURE(
          void,
          KinematicsInterface,
          initialize,
          planner_setting
        );
      }

      void displayPosture(const KinematicsState& kin_state, double time_step) override
      {
        PYBIND11_OVERLOAD_PURE(
          void,
          KinematicsInterface,
		  displayPosture,
          kin_state,
		  time_step
        );
      }

      KinematicsState updateJacobiansAndState(KinematicsState& kin_state, double dt) override
      {
        PYBIND11_OVERLOAD_PURE(
          KinematicsState,
          KinematicsInterface,
		  updateJacobiansAndState,
          kin_state,
		  dt
        );
      }

      KinematicsState integratePosture(KinematicsState& kin_state, double dt) override
      {
        PYBIND11_OVERLOAD_PURE(
          KinematicsState,
          KinematicsInterface,
		  integratePosture,
		  kin_state,
		  dt
        );
      }

      KinematicsState differentiatePostures(KinematicsState& start_state, KinematicsState& end_state, double timestep) override
      {
        PYBIND11_OVERLOAD_PURE(
          KinematicsState,
          KinematicsInterface,
		  differentiatePostures,
		  start_state,
		  end_state,
		  timestep
        );
      }

      Eigen::Vector3d logarithmicMap(const Eigen::Vector4d quat_wxyz) override
      {
        PYBIND11_OVERLOAD_PURE(
          Eigen::Vector3d,
          KinematicsInterface,
          logarithmicMap,
          quat_wxyz
        );
      }
  };

}
