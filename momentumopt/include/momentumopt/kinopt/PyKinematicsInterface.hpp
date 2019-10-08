/**
 * @file PyKinematicsInterface.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
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
