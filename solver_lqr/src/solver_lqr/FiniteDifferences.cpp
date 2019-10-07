/**
 * @file FiniteDifferences.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <yaml_cpp_catkin/yaml_eigen.h>
#include <solver_lqr/FiniteDifferences.hpp>

namespace solverlqr
{
  void FiniteDifferences::initialize(OcpBase* ocp_description,
		  bool compute_objective_first_derivatives, bool compute_dynamics_first_derivatives,
		  bool compute_objective_second_derivatives, bool compute_dynamics_second_derivatives)
  {
	if (!is_initialized_) {
	  is_initialized_ = true;
	  ocp_ = ocp_description;
	  compute_dynamics_first_derivatives_ = compute_dynamics_first_derivatives;
	  compute_dynamics_second_derivatives_ = compute_dynamics_second_derivatives;
	  compute_objective_first_derivatives_  = compute_objective_first_derivatives;
	  compute_objective_second_derivatives_  = compute_objective_second_derivatives;

	  tdim_ = ocp_->tdim();
	  xdim_ = ocp_->xdim();
	  udim_ = ocp_->udim();

      // resizing storages
	  std::vector<Eigen::MatrixXd> fxx, fxu, fuu;
	  for (int id=0; id<xdim_; id++) { fxx.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_)); }
	  for (int id=0; id<udim_; id++) { fxu.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_)); }
	  for (int id=0; id<udim_; id++) { fuu.push_back(Eigen::MatrixXd::Zero(xdim_, udim_)); }

      for (int time=0; time<tdim_; time++) {
        fxx_seq_.push_back(fxx);
        fxu_seq_.push_back(fxu);
        fuu_seq_.push_back(fuu);
        fx_seq_.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_));
        fu_seq_.push_back(Eigen::MatrixXd::Zero(xdim_, udim_));
      }

      for (int time=0; time<tdim_+1; time++) {
        cx_seq_.push_back(Eigen::VectorXd::Zero(xdim_, 1));
        cu_seq_.push_back(Eigen::VectorXd::Zero(udim_, 1));
        cxx_seq_.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_));
        cxu_seq_.push_back(Eigen::MatrixXd::Zero(xdim_, udim_));
        cuu_seq_.push_back(Eigen::MatrixXd::Zero(udim_, udim_));
      }
    }
  }

  void FiniteDifferences::computeDerivatives()
  {
    if (compute_dynamics_first_derivatives_ && is_initialized_) { this->computeDynamicsFirstDerivatives(); }
    if (compute_dynamics_second_derivatives_ && is_initialized_) { this->computeDynamicsSecondDerivatives(); }
    if (compute_objective_first_derivatives_ && is_initialized_) { this->computeObjectiveFirstDerivatives(); }
    if (compute_objective_second_derivatives_ && is_initialized_) { this->computeObjectiveSecondDerivatives(); }
  }

  void FiniteDifferences::computeDynamicsFirstDerivatives()
  {
	ControlBase A(xdim_, udim_);
	StateBase S(xdim_), Sp(xdim_), Sm(xdim_);

    // sequence of dynamics first derivatives
    for (int time_id=0; time_id<tdim_; time_id++)
    {
      S = ocp_->stateSeq().state(time_id);
      A = ocp_->controlSeq().control(time_id);

      // computing fx
      for (int row=0; row<xdim_; row++)
      {
        S.stateVector()[row] += h_devs_;  ocp_->internal_dynamics(S,A,Sp,time_id);  S.stateVector()[row] -= h_devs_;
        S.stateVector()[row] -= h_devs_;  ocp_->internal_dynamics(S,A,Sm,time_id);  S.stateVector()[row] += h_devs_;
        fx_seq_[time_id].col(row) = (Sp-Sm).stateVector().matrix()/(2.*h_devs_);
      }

      // computing fu
      for (int row=0; row<udim_; row++)
      {
        A.feedforward()[row] += h_devs_;  ocp_->internal_dynamics(S,A,Sp,time_id);  A.feedforward()[row] -= h_devs_;
        A.feedforward()[row] -= h_devs_;  ocp_->internal_dynamics(S,A,Sm,time_id);  A.feedforward()[row] += h_devs_;
        fu_seq_[time_id].col(row) = (Sp-Sm).stateVector().matrix()/(2.*h_devs_);
      }
    }
  }

  void FiniteDifferences::computeDynamicsSecondDerivatives()
  {
	ControlBase Ai(xdim_, udim_), Aj(xdim_, udim_);
	StateBase Si(xdim_), Sj(xdim_), Spp(xdim_), Spm(xdim_), Smp(xdim_), Smm(xdim_);

    // sequence of dynamics second derivatives
    for (int time_id=0; time_id<tdim_; time_id++)
    {
      Si = ocp_->stateSeq().state(time_id);
      Sj = ocp_->stateSeq().state(time_id);
      Ai = ocp_->controlSeq().control(time_id);
      Aj = ocp_->controlSeq().control(time_id);

      // computing fxx
      for (int row=0; row<xdim_; row++)
      {
        Si.stateVector()[row] += h_devs_;   Sj.stateVector()[row] -= h_devs_;
        for (int col=0; col<xdim_; col++)
        {
          Si.stateVector()[col] += h_devs_;   ocp_->internal_dynamics(Si,Ai,Spp,time_id);   Si.stateVector()[col] -= h_devs_;
          Si.stateVector()[col] -= h_devs_;   ocp_->internal_dynamics(Si,Ai,Spm,time_id);   Si.stateVector()[col] += h_devs_;
          Sj.stateVector()[col] += h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smp,time_id);   Sj.stateVector()[col] -= h_devs_;
          Sj.stateVector()[col] -= h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smm,time_id);   Sj.stateVector()[col] += h_devs_;
          fxx_seq_[time_id][row].col(col) = (Spp-Spm-Smp+Smm).stateVector()/(4.*std::pow(h_devs_,2.));
        }
        Si.stateVector()[row] -= h_devs_;   Sj.stateVector()[row] += h_devs_;
      }

      // computing fxu
      for (int row=0; row<udim_; row++)
      {
        Ai.feedforward()[row] += h_devs_;   Aj.feedforward()[row] -= h_devs_;
        for (int col=0; col<xdim_; col++)
        {
          Si.stateVector()[col] += h_devs_;   ocp_->internal_dynamics(Si,Ai,Spp,time_id);   Si.stateVector()[col] -= h_devs_;
          Si.stateVector()[col] -= h_devs_;   ocp_->internal_dynamics(Si,Ai,Spm,time_id);   Si.stateVector()[col] += h_devs_;
          Sj.stateVector()[col] += h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smp,time_id);   Sj.stateVector()[col] -= h_devs_;
          Sj.stateVector()[col] -= h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smm,time_id);   Sj.stateVector()[col] += h_devs_;
          fxu_seq_[time_id][row].col(col) = (Spp-Spm-Smp+Smm).stateVector()/(4.*std::pow(h_devs_,2.));
        }
        Ai.feedforward()[row] -= h_devs_;   Aj.feedforward()[row] += h_devs_;
      }

      // computing fuu
      for (int row=0; row<udim_; row++)
      {
        Ai.feedforward()[row] += h_devs_;   Aj.feedforward()[row] -= h_devs_;
        for (int col=0; col<udim_; col++)
        {
          Ai.feedforward()[col] += h_devs_;   ocp_->internal_dynamics(Si,Ai,Spp,time_id);   Ai.feedforward()[col] -= h_devs_;
          Ai.feedforward()[col] -= h_devs_;   ocp_->internal_dynamics(Si,Ai,Spm,time_id);   Ai.feedforward()[col] += h_devs_;
          Aj.feedforward()[col] += h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smp,time_id);   Aj.feedforward()[col] -= h_devs_;
          Aj.feedforward()[col] -= h_devs_;   ocp_->internal_dynamics(Sj,Aj,Smm,time_id);   Aj.feedforward()[col] += h_devs_;
          fuu_seq_[time_id][row].col(col) = (Spp-Spm-Smp+Smm).stateVector()/(4.*std::pow(h_devs_,2.));
        }
        Ai.feedforward()[row] -= h_devs_;   Aj.feedforward()[row] += h_devs_;
      }
    }
  }

  void FiniteDifferences::computeObjectiveFirstDerivatives()
  {
	double cp, cm;
	StateBase S(xdim_);
    ControlBase A(xdim_, udim_);

    // sequence of objective first derivatives
    for (int time=0; time<=tdim_; time++)
    {
      S = ocp_->stateSeq().state(time);
      if (time<tdim_)
        A = ocp_->controlSeq().control(time);
      else
    	A.setZero();

      // computing cx
      for (int row=0; row<xdim_; row++)
      {
        S.stateVector()[row] += h_devs_;  cp = ocp_->objective(S,A,time,time==tdim_);  S.stateVector()[row] -= h_devs_;
        S.stateVector()[row] -= h_devs_;  cm = ocp_->objective(S,A,time,time==tdim_);  S.stateVector()[row] += h_devs_;
        cx_seq_[time](row) = (cp-cm)/(2.*h_devs_);
      }

      // computing cu
      for (int row=0; row<udim_; row++)
      {
        A.feedforward()[row] += h_devs_;  cp = ocp_->objective(S,A,time,time==tdim_);  A.feedforward()[row] -= h_devs_;
        A.feedforward()[row] -= h_devs_;  cm = ocp_->objective(S,A,time,time==tdim_);  A.feedforward()[row] += h_devs_;
        cu_seq_[time](row) = (cp-cm)/(2.*h_devs_);
      }
    }
  }

  void FiniteDifferences::computeObjectiveSecondDerivatives()
  {
	double cpp, cpm, cmp, cmm;
	StateBase Si(xdim_), Sj(xdim_);
    ControlBase Ai(xdim_, udim_), Aj(xdim_, udim_);

    // sequence of objective second derivatives
    for (int time=0; time<=tdim_; time++)
    {
      Si = ocp_->stateSeq().state(time);
      Sj = ocp_->stateSeq().state(time);
      if (time<tdim_) {
        Ai = ocp_->controlSeq().control(time);
        Aj = ocp_->controlSeq().control(time);
      } else {
    	Ai.setZero();
    	Aj.setZero();
      }

      // computing cxx
      for (int row=0; row<xdim_; row++)
      {
        Si.stateVector()[row] += h_devs_;   Sj.stateVector()[row] -= h_devs_;
        for (int col=0; col<xdim_; col++)
        {
          Si.stateVector()[col] += h_devs_;  cpp = ocp_->objective(Si,Ai,time,time==tdim_);  Si.stateVector()[col] -= h_devs_;
          Si.stateVector()[col] -= h_devs_;  cpm = ocp_->objective(Si,Ai,time,time==tdim_);  Si.stateVector()[col] += h_devs_;
          Sj.stateVector()[col] += h_devs_;  cmp = ocp_->objective(Sj,Aj,time,time==tdim_);  Sj.stateVector()[col] -= h_devs_;
          Sj.stateVector()[col] -= h_devs_;  cmm = ocp_->objective(Sj,Aj,time,time==tdim_);  Sj.stateVector()[col] += h_devs_;
          cxx_seq_[time](row,col) = (cpp-cpm-cmp+cmm)/(4.*std::pow(h_devs_,2.));
        }
        Si.stateVector()[row] -= h_devs_;   Sj.stateVector()[row] += h_devs_;
      }

      // computing cxu
      for (int row=0; row<xdim_; row++)
      {
        Si.stateVector()[row] += h_devs_;   Sj.stateVector()[row] -= h_devs_;
        for (int col=0; col<udim_; col++)
        {
          Ai.feedforward()[col] += h_devs_;  cpp = ocp_->objective(Si,Ai,time,time==tdim_);  Ai.feedforward()[col] -= h_devs_;
          Ai.feedforward()[col] -= h_devs_;  cpm = ocp_->objective(Si,Ai,time,time==tdim_);  Ai.feedforward()[col] += h_devs_;
          Aj.feedforward()[col] += h_devs_;  cmp = ocp_->objective(Sj,Aj,time,time==tdim_);  Aj.feedforward()[col] -= h_devs_;
          Aj.feedforward()[col] -= h_devs_;  cmm = ocp_->objective(Sj,Aj,time,time==tdim_);  Aj.feedforward()[col] += h_devs_;
          cxu_seq_[time](row,col) = (cpp-cpm-cmp+cmm)/(4.*std::pow(h_devs_,2.));
        }
        Si.stateVector()[row] -= h_devs_;   Sj.stateVector()[row] += h_devs_;
      }

      // computing cuu
      for (int row=0; row<udim_; row++)
      {
        Ai.feedforward()[row] += h_devs_;   Aj.feedforward()[row] -= h_devs_;
        for (int col=0; col<udim_; col++)
        {
          Ai.feedforward()[col] += h_devs_;  cpp = ocp_->objective(Si,Ai,time,time==tdim_);  Ai.feedforward()[col] -= h_devs_;
          Ai.feedforward()[col] -= h_devs_;  cpm = ocp_->objective(Si,Ai,time,time==tdim_);  Ai.feedforward()[col] += h_devs_;
          Aj.feedforward()[col] += h_devs_;  cmp = ocp_->objective(Sj,Aj,time,time==tdim_);  Aj.feedforward()[col] -= h_devs_;
          Aj.feedforward()[col] -= h_devs_;  cmm = ocp_->objective(Sj,Aj,time,time==tdim_);  Aj.feedforward()[col] += h_devs_;
          cuu_seq_[time](row,col) = (cpp-cpm-cmp+cmm)/(4.*std::pow(h_devs_,2.));
        }
        Ai.feedforward()[row] -= h_devs_;   Aj.feedforward()[row] += h_devs_;
      }
    }
  }

}
