/**
 * @file OcpDescription.hpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <solver_lqr/SolverLqrSetting.hpp>

namespace solverlqr {

  //! Class to define State of OCP
  class StateBase
  {
    public:
	  StateBase(){}
	  StateBase(int xdim) { state_ = Eigen::VectorXd(xdim).setZero(); }
	  ~StateBase(){}

	  StateBase& operator= (const StateBase& rhs) { state_ = rhs.stateVector(); return *this; }
	  StateBase  operator* (double scalar) const { StateBase new_state; new_state.stateVector() = scalar*state_; return new_state; }
	  StateBase& operator*=(double scalar) { state_ *= scalar; return *this; }
	  StateBase  operator+ (const StateBase& rhs) const { StateBase new_state; new_state.stateVector() = state_+rhs.stateVector(); return new_state; }
	  StateBase& operator+=(const StateBase& rhs) { state_ += rhs.stateVector(); return *this; }
	  StateBase  operator/ (double scalar) const { StateBase new_state; new_state.stateVector() = state_/scalar; return new_state; }
	  StateBase& operator/=(double scalar) { state_ /= scalar; return *this; }

      Eigen::VectorXd& stateVector() { return state_; }
      const Eigen::VectorXd& stateVector() const { return state_; }
      void resize(int xdim) { state_ = Eigen::VectorXd(xdim).setZero(); }

    private:
      Eigen::VectorXd state_;
  };
  StateBase operator* (double scalar, const StateBase& rhs);
  StateBase operator- (const StateBase& lhs, const StateBase& rhs);


  class StateSequence
  {
    public:
      StateSequence(){}
      StateSequence(int tdim, int xdim) { this->resize(tdim, xdim); }
      ~StateSequence(){}

      void resize(int tdim, int xdim);
      void setRandom(double scaling = 1.0);
      const int size() const { return stateseq_.size(); }
      StateBase& state(int id) { return stateseq_[id]; }
      const StateBase& state(int id) const { return stateseq_[id]; }

    private:
      std::vector<StateBase> stateseq_;
  };

  //! Class to define Control of OCP
  class ControlBase
  {
    public:
      ControlBase(){}
      ControlBase(int xdim, int udim);
      ~ControlBase(){}

      ControlBase& operator= (const ControlBase& rhs);
      ControlBase  operator* (double scalar) const;
      ControlBase& operator*=(double scalar);
      ControlBase  operator+ (const ControlBase& rhs) const;
      ControlBase& operator+=(const ControlBase& rhs);

      void setZero() { feedforward_.setZero(); }
      Eigen::MatrixXd& feedback() { return feedback_; }
      Eigen::VectorXd& feedforward() { return feedforward_; }
      const Eigen::MatrixXd& feedback() const { return feedback_; }
      const Eigen::VectorXd& feedforward() const { return feedforward_; }
      void setRandom(double scaling = 1.0) { feedforward_.setRandom(); feedforward_ *= scaling; }

    private:
      Eigen::MatrixXd feedback_;
      Eigen::VectorXd feedforward_;
  };
  ControlBase operator*(double lhs_scalar, const ControlBase& rhs);

  class ControlSequence
  {
    public:
      ControlSequence(){}
      ControlSequence(int tdim, int xdim, int udim) { this->resize(tdim, xdim, udim); }
      ~ControlSequence(){}

      void setRandom(double scaling = 1.0);
      void resize(int tdim, int xdim, int udim);
      const int size() const { return controlseq_.size(); }
      ControlBase& control(int id) { return controlseq_[id]; }
      const ControlBase& control(int id) const { return controlseq_[id]; }

      ControlSequence& operator= (const ControlSequence& rhs);
      ControlSequence  operator* (double scalar) const;
      ControlSequence& operator*=(double scalar);
      ControlSequence  operator+ (const ControlSequence& rhs) const;
      ControlSequence& operator+=(const ControlSequence& rhs);

      static double controlSequenceGradientNorm(const ControlSequence& u1, const ControlSequence& u2);

    private:
      std::vector<ControlBase> controlseq_;
  };
  ControlSequence operator*(double lhs_scalar, const ControlSequence& rhs);

  //! Optimal Control Problem Description Class
  class OcpBase
  {
    public:
	  OcpBase(){}
      ~OcpBase(){}
      void initialize(const SolverLqrSetting& setting);

      const double& dt() const { return this->getLqrSetting().get(SolverLqrDoubleParam_TimeStep); }
      const int& tdim() const { return this->getLqrSetting().get(SolverLqrIntParam_TimeDimension); }
      const int& xdim() const { return this->getLqrSetting().get(SolverLqrIntParam_StateDimension); }
      const int& udim() const { return this->getLqrSetting().get(SolverLqrIntParam_ControlDimension); }

      StateSequence& stateSeq() { return stateseq_; }
      ControlSequence& controlSeq() { return controlseq_; }
      const StateSequence& stateSeq() const { return stateseq_; }
      const ControlSequence& controlSeq() const { return controlseq_; }
      const SolverLqrSetting& getLqrSetting() const { return *setting_; }

      virtual void configure(const YAML::Node& user_parameters) = 0;
      virtual Eigen::MatrixXd processNoiseFilter(int time_id) const;
      virtual Eigen::MatrixXd measurementNoiseFilter(int time_id) const;
      virtual StateBase dynamics(const StateBase& state, const ControlBase& control, int time_id) = 0;
      void internal_dynamics(const StateBase& state, const ControlBase& control, StateBase& new_state, int time_id);
      virtual double objective(const StateBase& state, const ControlBase& control, int time_id, bool is_final_timestep) = 0;

    private:
      friend class Estimator;
      friend class ForwardPass;
      friend class BackwardPass;
      friend class FiniteDifferences;

    private:
      StateSequence stateseq_;
      ControlSequence controlseq_;
      const SolverLqrSetting* setting_;
  };

}
