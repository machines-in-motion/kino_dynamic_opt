/**
 * @file OcpDescription.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <iostream>
#include <yaml_utils/yaml_eigen.hpp>
#include <solver_lqr/OcpDescription.hpp>

namespace solverlqr
{
  StateBase operator* (double scalar, const StateBase& rhs) { StateBase new_state; new_state.stateVector() = scalar*rhs.stateVector(); return new_state; }
  StateBase operator- (const StateBase& lhs, const StateBase& rhs) { StateBase new_state; new_state.stateVector() = lhs.stateVector()-rhs.stateVector(); return new_state; }

  // ControlBase functions
  ControlBase::ControlBase(int xdim, int udim)
  {
	this->feedforward().resize(udim);  this->feedforward().setZero();
	this->feedback().resize(udim, xdim);  this->feedback().setZero();
  }

  ControlBase& ControlBase::operator=(const ControlBase& rhs)
  {
    this->feedback() = rhs.feedback();
    this->feedforward() = rhs.feedforward();
    return *this;
  }

  ControlBase ControlBase::operator*(double scalar) const
  {
    ControlBase lhs;
    lhs.feedback() = this->feedback();
    lhs.feedforward() = this->feedforward() * scalar;
    return lhs;
  }

  ControlBase& ControlBase::operator*=(double scalar)
  {
    this->feedforward() *= scalar;
	return *this;
  }

  ControlBase operator*(double lhs_scalar, const ControlBase& rhs)
  {
    return rhs*lhs_scalar;
  }

  ControlBase ControlBase::operator+(const ControlBase& rhs) const
  {
    ControlBase lhs;
    lhs.feedback() = rhs.feedback();
    lhs.feedforward() = this->feedforward() + rhs.feedforward();
    return lhs;
  }

  ControlBase& ControlBase::operator+=(const ControlBase& rhs)
  {
    this->feedback() = rhs.feedback();
    this->feedforward() += rhs.feedforward();
	return *this;
  }

  // StateSequence functions
  void StateSequence::resize(int tdim, int xdim)
  {
	StateBase state(xdim);
	for (int time=0; time<tdim; time++)
	  stateseq_.push_back(state);
  }

  void StateSequence::setRandom(double scaling)
  {
	for (int time=0; time<this->size(); time++) {
	  this->state(time).stateVector().setRandom();
	  this->state(time).stateVector() *= scaling;
	}
  }

  // ControlSequence functions
  void ControlSequence::resize(int tdim, int xdim, int udim)
  {
	controlseq_.clear();
	ControlBase control(xdim, udim);
	for (int time=0; time<tdim; time++)
	  controlseq_.push_back(control);
  }

  void ControlSequence::setRandom(double scaling)
  {
	for (int time=0; time<this->size(); time++)
	  this->control(time).setRandom(scaling);
  }

  ControlSequence& ControlSequence::operator= (const ControlSequence& rhs)
  {
	this->resize(rhs.size(), rhs.control(0).feedback().cols(), rhs.control(0).feedback().rows());
	for (int time=0; time<rhs.size(); time++)
      this->control(time) = rhs.control(time);
    return *this;
  }

  ControlSequence ControlSequence::operator*(double scalar) const
  {
	ControlSequence lhs = *this;
	for (int time=0; time<this->size(); time++)
	  lhs.control(time) *= scalar;
    return lhs;
  }

  ControlSequence& ControlSequence::operator*=(double scalar)
  {
    for (int time=0; time<this->size(); time++)
      this->control(time) *= scalar;
	return *this;
  }

  ControlSequence operator*(double lhs_scalar, const ControlSequence& rhs)
  {
    return rhs*lhs_scalar;
  }

  ControlSequence ControlSequence::operator+(const ControlSequence& rhs) const
  {
	ControlSequence lhs = *this;
	lhs += rhs;
    return lhs;
  }

  ControlSequence& ControlSequence::operator+=(const ControlSequence& rhs)
  {
    for (int time=0; time<this->size(); time++)
      this->control(time) += rhs.control(time);
	return *this;
  }

  double ControlSequence::controlSequenceGradientNorm(const ControlSequence& u1, const ControlSequence& u2)
  {
	double grad_norm = 0.0;
    for (int time=0; time<u1.size(); time++)
      grad_norm += ( u1.control(time).feedforward().array().abs() /
                   ( u2.control(time).feedforward().array().abs()+1.) ).matrix().maxCoeff();
    return grad_norm / u1.size();
  }

  // OcpBase functions
  void OcpBase::initialize(const SolverLqrSetting& setting)
  {
    setting_ = &setting;
    this->stateSeq().resize(this->tdim()+1, this->xdim());
    this->controlSeq().resize(this->tdim(), this->xdim(), this->udim());
    this->configure(this->getLqrSetting().get(SolverLqrYamlParam_UserParameters));
  }

  void OcpBase::internal_dynamics(const StateBase& state, const ControlBase& control, StateBase& new_state, int time_id)
  {
    StateBase k1 = this->dynamics(state, control, time_id);
    StateBase dstate = k1;
    if (this->getLqrSetting().get(SolverLqrBoolParam_UseRungeKuttaIntegration)) {
      StateBase k2 = this->dynamics(state+0.5*k1, control, time_id);
      StateBase k3 = this->dynamics(state+0.5*k2, control, time_id);
      StateBase k4 = this->dynamics(state+k3    , control, time_id);
      dstate = (k1+k4)/6.0 + (k2+k3)/3.0;
    }
    new_state = state + dstate;
  }

  Eigen::MatrixXd OcpBase::processNoiseFilter(int /*time_id*/) const
  {
    Eigen::MatrixXd process_noise_filter(this->xdim(),this->xdim());
    process_noise_filter.setIdentity();
    return process_noise_filter;
  }

  Eigen::MatrixXd OcpBase::measurementNoiseFilter(int /*time_id*/) const
  {
    Eigen::MatrixXd measurement_noise_filter(this->xdim(),this->xdim());
    measurement_noise_filter.setIdentity();
    return measurement_noise_filter;
  }

}
