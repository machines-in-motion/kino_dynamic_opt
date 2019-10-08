/**
 * @file DynamicsFeedback.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-08
 */

#include <yaml_cpp_catkin/yaml_eigen.h>
#include <momentumopt/dynopt/DynamicsFeedback.hpp>

using namespace solverlqr;

namespace momentumopt {

  // DynamicsFeedback functions implementation
  void DynamicsFeedback::setStatesAndControls(const DynamicsState& ini_state, const DynamicsSequence& dyn_sequence)
  {
    ref_dyn_sequence_ = &dyn_sequence;

    this->stateSeq().state(0).stateVector().segment<3>(0) = ini_state.centerOfMass();
    this->stateSeq().state(0).stateVector().segment<3>(3) = ini_state.linearMomentum();
    this->stateSeq().state(0).stateVector().segment<3>(6) = ini_state.angularMomentum();

    if (this->tdim() != this->dynamicsSequence().size()) {
        throw std::runtime_error("LQR and trajectory optimization size do not match. "
                "Check if the LQR time_step and time_horizon match the dynamics parameters.");
    }

    for (int time_id=0; time_id<this->dynamicsSequence().size(); time_id++) {
      this->controlSeq().control(time_id).feedforward().setZero();
      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
        if (this->dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
          this->controlSeq().control(time_id).feedforward().segment<3>(3*eff_id) =  this->getSetting().get(PlannerDoubleParam_MassTimesGravity)*this->dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id);
    }
  }

  void DynamicsFeedback::configure(const YAML::Node& user_parameters)
  {
    try
    {
      control_cost_ = user_parameters["control_cost"].as<Eigen::VectorXd>().asDiagonal();
      com_tracking_ = user_parameters["com_tracking"].as<Eigen::VectorXd>().asDiagonal();
      lmom_tracking_ = user_parameters["lmom_tracking"].as<Eigen::VectorXd>().asDiagonal();
      amom_tracking_ = user_parameters["amom_tracking"].as<Eigen::VectorXd>().asDiagonal();
      com_final_tracking_ = user_parameters["com_final_tracking"].as<Eigen::VectorXd>().asDiagonal();
      lmom_final_tracking_ = user_parameters["lmom_final_tracking"].as<Eigen::VectorXd>().asDiagonal();
      amom_final_tracking_ = user_parameters["amom_final_tracking"].as<Eigen::VectorXd>().asDiagonal();
    }
    catch (std::runtime_error& e)
    {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl;
    }
  }

  double DynamicsFeedback::objective(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id, bool is_final_timestep)
  {
	Eigen::MatrixXd com_tracking  = com_tracking_,
                    lmom_tracking = lmom_tracking_,
                    amom_tracking = amom_tracking_;
	if (is_final_timestep) {
      com_tracking  = com_final_tracking_;
      lmom_tracking = lmom_final_tracking_;
      amom_tracking = amom_final_tracking_;
	}

	double objective = 0.0;
    for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
      if (this->dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
        objective += control.feedforward().segment<3>(3*eff_id).dot( control_cost_*(control.feedforward().segment<3>(3*eff_id)) );

    if (time_id>0) {
      objective += (state.stateVector().segment<3>(0)-this->dynamicsSequence().dynamicsState(time_id-1).centerOfMass()).dot( com_tracking*(state.stateVector().segment<3>(0)-this->dynamicsSequence().dynamicsState(time_id-1).centerOfMass()) );
      objective += (state.stateVector().segment<3>(3)-this->dynamicsSequence().dynamicsState(time_id-1).linearMomentum()).dot( lmom_tracking*(state.stateVector().segment<3>(3)-this->dynamicsSequence().dynamicsState(time_id-1).linearMomentum()) );
      objective += (state.stateVector().segment<3>(6)-this->dynamicsSequence().dynamicsState(time_id-1).angularMomentum()).dot( amom_tracking*(state.stateVector().segment<3>(6)-this->dynamicsSequence().dynamicsState(time_id-1).angularMomentum()) );
    } else {
      objective += (state.stateVector().segment<3>(0)-this->dynamicsSequence().dynamicsState(0).centerOfMass()).dot( com_tracking*(state.stateVector().segment<3>(0)-this->dynamicsSequence().dynamicsState(0).centerOfMass()) );
      objective += (state.stateVector().segment<3>(3)-this->dynamicsSequence().dynamicsState(0).linearMomentum()).dot( lmom_tracking*(state.stateVector().segment<3>(3)-this->dynamicsSequence().dynamicsState(0).linearMomentum()) );
      objective += (state.stateVector().segment<3>(6)-this->dynamicsSequence().dynamicsState(0).angularMomentum()).dot( amom_tracking*(state.stateVector().segment<3>(6)-this->dynamicsSequence().dynamicsState(0).angularMomentum()) );
    }

    return objective;
  }

  solverlqr::StateBase DynamicsFeedback::dynamics(const solverlqr::StateBase& state, const solverlqr::ControlBase& control, int time_id)
  {
    solverlqr::StateBase dstate(this->xdim());
    double dt = this->dynamicsSequence().dynamicsState(time_id).time();

    // change in linear momentum
    dstate.stateVector().segment<3>(3) = dt * this->getSetting().get(PlannerDoubleParam_RobotMass) * this->getSetting().get(PlannerVectorParam_GravityVector);
    for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
      if (this->dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
        dstate.stateVector().segment<3>(3) += dt * control.feedforward().segment<3>(3*eff_id);

    // change in center of mass
    dstate.stateVector().segment<3>(0) = dt / this->getSetting().get(PlannerDoubleParam_RobotMass) * (state.stateVector().segment<3>(3) + dstate.stateVector().segment<3>(3));

    // change in angular momentum
    dstate.stateVector().segment<3>(6).setZero();
    for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
      if (this->dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)){
        dstate.stateVector().segment<3>(6) += dt * (this->dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id) -
                                              (state.stateVector().segment<3>(0)+dstate.stateVector().segment<3>(0))).cross( control.feedforward().segment<3>(3*eff_id+0) );
      }
    return dstate;
  }

  // DynamicsFeedbackWrapper functions implementation
  void DynamicsFeedbackWrapper::initialize(SolverLqrSetting& lqr_stg, const PlannerSetting& plan_stg)
  {
    lqr_stg_ = &lqr_stg;
    plan_stg_ = &plan_stg;

    dynamics_description_.initialize(lqr_stg);
    dynamics_description_.setPlannerSetting(plan_stg);
    dynamics_lqrsolver_.initialize(&dynamics_description_, lqr_stg);
  }

  void DynamicsFeedbackWrapper::optimize(const DynamicsState& ini_state, const DynamicsSequence& dynamics_sequence)
  {
    dynamics_description_.setStatesAndControls(ini_state, dynamics_sequence);
    dynamics_lqrsolver_.optimize();
  }

  Eigen::MatrixXd DynamicsFeedbackWrapper::forceGain(int time_id)
  {
    int constrained_time_id = std::max(std::min(time_id, dynamics_description_.tdim()-1), 0);
    return dynamics_description_.controlSeq().control(constrained_time_id).feedback();
  }

}
