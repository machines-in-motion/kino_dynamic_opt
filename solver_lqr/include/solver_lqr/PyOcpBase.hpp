/**
 * @file PyOcpBase.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
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
