/**
 * @file BackwardPass.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <iostream>
#include <yaml_utils/yaml_eigen.hpp>
#include <solver_lqr/BackwardPass.hpp>

namespace solverlqr
{
  void BackwardPass::initialize(OcpBase* ocp, const SolverLqrSetting& stgs)
  {
    ocp_ = ocp;
    stgs_ = &stgs;
    tdim_ = ocp->tdim();
    xdim_ = ocp->xdim();
    udim_ = ocp->udim();
    is_initialized_ = true;
    control_seq_.resize(tdim_, xdim_, udim_);

    V_.resize(tdim_+1);          V_.setZero();
    Qu_.resize(udim_);           Qu_.setZero();
    Qx_.resize(xdim_);           Qx_.setZero();
    Qxx_.resize(xdim_, xdim_);   Qxx_.setZero();
    Qxu_.resize(xdim_, udim_);   Qxu_.setZero();
    Quu_.resize(udim_, udim_);   Quu_.setZero();
    VxxR_.resize(xdim_, xdim_);  VxxR_.setZero();
    QxuR_.resize(xdim_, udim_);  QxuR_.setZero();
    QuuR_.resize(udim_, udim_);  QuuR_.setZero();

    for (int time_id=0; time_id<=tdim_; time_id++) {
      Vx_.push_back(Eigen::VectorXd::Zero(xdim_));
      Vxh_.push_back(Eigen::VectorXd::Zero(xdim_));
      Vxx_.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_));
      Vxxh_.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_));
      Vxhxh_.push_back(Eigen::MatrixXd::Zero(xdim_, xdim_));
    }

    std::string cfg_file = CFG_PATH + std::string("default_solver_env.yaml");
    model_.getSetting().initialize(cfg_file);
  }

  void BackwardPass::optimizeController(const ForwardPass& fpass, const FiniteDifferences& devs, double lambda)
  {
    // Initialize backward pass
    dV_.setZero();
    diverge_flag_ = false;
    diverge_iteration_ = -1;
    Vx_[tdim_] = devs.cx(tdim_);
    Vxx_[tdim_] = devs.cxx(tdim_);

    V_[tdim_] = fpass.timestepCost(tdim_);
    for (int time=tdim_-1; time>=0; time--)
    {
      // renaming some variables for simplicity
      double ss = V_[time+1];
      Eigen::VectorXd sx = Vx_[time+1];
      Eigen::MatrixXd Sxx = Vxx_[time+1];

      Eigen::MatrixXd Q = devs.cxx(time);    Eigen::MatrixXd R = devs.cuu(time);    Eigen::MatrixXd P = devs.cxu(time).transpose();
      Eigen::VectorXd q = devs.cx(time);     Eigen::VectorXd r = devs.cu(time);     double qs = fpass.timestepCost(time);
      Eigen::MatrixXd A = devs.fx(time);     Eigen::MatrixXd B = devs.fu(time);

      // compute auxiliar terms
      Eigen::VectorXd g = r + B.transpose()*sx;
      Eigen::MatrixXd G = P + B.transpose()*Sxx*A;
      Eigen::MatrixXd H = R + B.transpose()*Sxx*B + lambda*Eigen::MatrixXd::Identity(udim_, udim_);

      // Cholesky decomposition and positive definiteness check
      llt_.compute(H);
      if (llt_.info() != Eigen::ComputationInfo::Success) {
        diverge_iteration_ = time;
        diverge_flag_ = true;
        break;
      }

      // Computing feedforward and feedback terms
      Eigen::VectorXd l = -llt_.solve(g);
      Eigen::MatrixXd L = -llt_.solve(G);
      control_seq_.control(time).feedback() = L;
      control_seq_.control(time).feedforward() = l;

      if (this->getLqrSetting().get(SolverLqrBoolParam_HasControlLimits))
        this->findConstrainedControls(time, llt_.matrixL(), g);

      // Computing quadratic approximation of the value function
      Eigen::MatrixXd Sxxn(xdim_, xdim_);
      V_[time]   = qs + ss + 0.5*l.transpose()*H*l +l.transpose()*g;
      Sxxn   = Q + A.transpose()*Sxx*A + L.transpose()*H*L + L.transpose()*G + G.transpose()*L;
      Vx_[time]  = q.transpose() + sx.transpose()*A + l.transpose()*H*L + l.transpose()*G + g.transpose()*L;
      Vxx_[time]   = 0.5*(Sxxn + Sxxn.transpose());
    }
  }

  Eigen::MatrixXd BackwardPass::tensorContraction(const Eigen::VectorXd& vz, const std::vector<Eigen::MatrixXd>& fzz)
  {
    int dima=fzz[0].cols(), dimb=fzz.size();
    Eigen::MatrixXd contraction(dima, dimb);  contraction.setZero();
    for (int indB=0; indB<dimb; indB++)
      for (int indA=0; indA<dima; indA++)
      contraction(indA,indB) = fzz[indB].col(indA).dot(vz);
	return contraction;
  }

  void BackwardPass::findConstrainedControls(int time_id, const Eigen::MatrixXd& chol_hessian, const Eigen::VectorXd& gradient)
  {
    vars_.clear();
    model_.clean();
    for (int var_id=0; var_id<udim_; var_id++) {
      vars_.push_back(solver::Var());
      vars_[var_id] = model_.addVar(solver::VarType::Continuous, 0.0, 1.0, 0.5);
    }

    quad_expr_.clear();
    for (int col_id=0; col_id<udim_; col_id++) {
      lin_expr_ = 0.0;
      for (int row_id=0; row_id<udim_; row_id++)
        lin_expr_ += chol_hessian(row_id,col_id)*vars_[row_id];
      quad_expr_.addQuaTerm(1.0, lin_expr_);
      quad_expr_.addLinTerm(gradient[col_id]*vars_[col_id]);
    }
    model_.setObjective(quad_expr_, 0.0);

    for (int ctrl_id=0; ctrl_id<udim_; ctrl_id++) {
      model_.addLinConstr(this->getOcp().controlSeq().control(time_id).feedforward()[ctrl_id]+vars_[ctrl_id], ">", this->getLqrSetting().get(SolverLqrVectorParam_MinControlLimits)[ctrl_id]);
      model_.addLinConstr(this->getOcp().controlSeq().control(time_id).feedforward()[ctrl_id]+vars_[ctrl_id], "<", this->getLqrSetting().get(SolverLqrVectorParam_MaxControlLimits)[ctrl_id]);
    }

    model_.optimize();

    for (int var_id=0; var_id<udim_; var_id++)
      control_seq_.control(time_id).feedforward()[var_id] = vars_[var_id].get(solver::SolverDoubleParam_X);

    double threshold = 0.01;
    for (int ctrl_id=0; ctrl_id<udim_; ctrl_id++) {
      if ((this->getOcp().controlSeq().control(time_id).feedforward()[ctrl_id]+control_seq_.control(time_id).feedforward()[ctrl_id] >= this->getLqrSetting().get(SolverLqrVectorParam_MaxControlLimits)[ctrl_id]-threshold) ||
          (this->getOcp().controlSeq().control(time_id).feedforward()[ctrl_id]+control_seq_.control(time_id).feedforward()[ctrl_id] <= this->getLqrSetting().get(SolverLqrVectorParam_MinControlLimits)[ctrl_id]+threshold))
        control_seq_.control(time_id).feedback().row(ctrl_id).setZero();
    }
  }

}
