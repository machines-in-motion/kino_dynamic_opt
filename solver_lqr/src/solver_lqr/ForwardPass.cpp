/**
 * @file ForwardPass.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <iostream>
#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/ForwardPass.hpp>

namespace solverlqr {

  void ForwardPass::initialize(OcpBase* ocp, const SolverLqrSetting& stgs)
  {
    ocp_ = ocp;
    stgs_ = &stgs;
  }

  void ForwardPass::applyController(const StateSequence& nom_state_seq, const ControlSequence& nom_control_seq)
  {
    // Initialize forward pass
    new_cost_ = 0.0;
    has_diverged_ = false;
    new_state_seq_ = nom_state_seq;
    new_control_seq_ = nom_control_seq;
    timestep_cost_.resize(this->getOcp().tdim()+1);

    // Forward propagate the dynamics
    for (int time_id=0; time_id<this->getOcp().tdim(); time_id++) {
      dx_.stateVector() = new_state_seq_.state(time_id).stateVector() - nom_state_seq.state(time_id).stateVector();
      new_control_seq_.control(time_id).feedforward() = nom_control_seq.control(time_id).feedforward() +
                                                        nom_control_seq.control(time_id).feedback()*dx_.stateVector();

      if (this->getLqrSetting().get(SolverLqrBoolParam_HasControlLimits)) {
        for (int ctrl_id=0; ctrl_id<this->getOcp().udim(); ctrl_id++) {
          if (new_control_seq_.control(time_id).feedforward()[ctrl_id] >= this->getLqrSetting().get(SolverLqrVectorParam_MaxControlLimits)[ctrl_id])
            new_control_seq_.control(time_id).feedforward()[ctrl_id] = this->getLqrSetting().get(SolverLqrVectorParam_MaxControlLimits)[ctrl_id];
          if (new_control_seq_.control(time_id).feedforward()[ctrl_id] <= this->getLqrSetting().get(SolverLqrVectorParam_MinControlLimits)[ctrl_id])
          new_control_seq_.control(time_id).feedforward()[ctrl_id] = this->getLqrSetting().get(SolverLqrVectorParam_MinControlLimits)[ctrl_id];
    	    }
      }

      this->getOcp().internal_dynamics(new_state_seq_.state(time_id), new_control_seq_.control(time_id), new_state_seq_.state(time_id+1), time_id);
      timestep_cost_[time_id] = this->getOcp().objective(new_state_seq_.state(time_id), new_control_seq_.control(time_id), time_id, false);

      // Divergence check
      for (int id=0; id<new_state_seq_.state(time_id+1).stateVector().size(); id++)
        if (std::abs(new_state_seq_.state(time_id+1).stateVector()[id]) > this->getLqrSetting().get(SolverLqrDoubleParam_DivergenceLimitCheck)) {
          has_diverged_  = true;
          break;
        }
	}
    timestep_cost_[this->getOcp().tdim()] = this->getOcp().objective(new_state_seq_.state(this->getOcp().tdim()),
                                                                              ControlBase(this->getOcp().xdim(), this->getOcp().udim()),
                                                                              this->getOcp().tdim(), true);
    new_cost_ = timestep_cost_.sum();
  }
}
