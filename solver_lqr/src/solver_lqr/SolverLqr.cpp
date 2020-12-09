/**
 * @file SolverLqr.cpp
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-07
 */

#include <fstream>
#include <iostream>
#include <yaml_utils/yaml_eigen.hpp>
#include <solver_lqr/SolverLqr.hpp>

namespace solverlqr {

  SolverLqr::SolverLqr()
    : converged_(true),
      devs_flag_(false),
      backpass_flag_(false),
      forwpass_flag_(false)
  {
  }

  void SolverLqr::initialize(OcpBase* ocp, SolverLqrSetting& setting)
  {
    ocp_ = ocp;
    setting_ = &setting;
    this->getPrinter().initialize(this->getLqrSetting());
    this->getForwPass().initialize(ocp_, this->getLqrSetting());
    this->getBackPass().initialize(ocp_, this->getLqrSetting());
    this->getFiniteDiff().initialize(ocp_, true, true, true, false);
  }

  void SolverLqr::optimize()
  {
    // Initialize algorithm
    this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) = this->getLqrSetting().get(SolverLqrDoubleParam_BackPassInitialRegularization);
    mult_regularization_change_ = this->getLqrSetting().get(SolverLqrDoubleParam_BackPassInitialMultRegularizationIncr);

    // Forward simulation of initial trajectory
    for (int i=0; i<this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs).size(); i++) {
      linesearch_coeff_ = this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs)[i];
      this->getForwPass().applyController(this->getOcp().stateSeq(), linesearch_coeff_*this->getOcp().controlSeq());
      this->getLqrInfo().get(SolverLqrDoubleParam_Cost) = this->getForwPass().newCost();
      if (!this->getForwPass().hasDiverged()) {
        this->getOcp().stateSeq() = this->getForwPass().stateSeq();
        this->getOcp().controlSeq() = this->getForwPass().controlSeq();
        break;
      }
    }

    if (this->getForwPass().hasDiverged()) {
      this->getPrinter().display(MsgLqr::EXIT_divergence, this->getLqrInfo());
      converged_ = false;
      return;
    }

    // Initialization of configuration variables
    devs_flag_ = true;
    this->getLqrInfo().get(SolverLqrDoubleParam_ExpectedCost) = 0.0;

    // Begin algorithm iterations
    this->getPrinter().display(MsgLqr::STAT_begin, this->getLqrInfo());

    for (this->getLqrInfo().get(SolverLqrIntParam_CurrentIteration)=0; this->getLqrInfo().get(SolverLqrIntParam_CurrentIteration)<this->getLqrSetting().get(SolverLqrIntParam_LqrMaxIterations); this->getLqrInfo().get(SolverLqrIntParam_CurrentIteration)++)
    {
      // ====== STEP 1: differentiate dynamics and cost along new trajectory
      if (devs_flag_)
      {
        this->getFiniteDiff().computeDerivatives();
        devs_flag_ = false;
      }

      // ====== STEP 2: backward pass, compute optimal control law
      backpass_flag_ = false;
      while (!backpass_flag_)
      {
        this->getBackPass().optimizeController(this->getForwPass(), this->getFiniteDiff(), this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization));
        backpass_flag_ = true;
        if (this->getBackPass().hasDiverged())
        {
          backpass_flag_ = false;
          this->getLqrInfo().get(SolverLqrIntParam_BackpassDivergeIteration) = this->getBackPass().divergeIteration();
          this->getPrinter().display(MsgLqr::STAT_cholesky, this->getLqrInfo());
          mult_regularization_change_ = std::max(mult_regularization_change_*this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr), this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr));
          this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) = std::max(this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization)*mult_regularization_change_, this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMinRegularization));
          if (this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) > this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMaxRegularization))
            break;
        }
      }

      // Check for termination due to small gradient
      this->getLqrInfo().get(SolverLqrDoubleParam_ControlGradient) = ControlSequence::controlSequenceGradientNorm(this->getBackPass().controlSeq(), this->getOcp().controlSeq());
      if (this->getLqrInfo().get(SolverLqrDoubleParam_ControlGradient) < this->getLqrSetting().get(SolverLqrDoubleParam_ControlGradientTolerance)) {
        this->getPrinter().display(MsgLqr::SUCC_gradient, this->getLqrInfo());
        break;
      }

      // ====== STEP 3: line-search to find new control sequence, trajectory, cost
      forwpass_flag_ = false;
      if (backpass_flag_)
      {
        for (int i=0; i<this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs).size(); i++)
        {
          linesearch_coeff_ = this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs)[i];
          this->getForwPass().applyController(this->getOcp().stateSeq(), this->getOcp().controlSeq() + linesearch_coeff_*this->getBackPass().controlSeq());
          this->getLqrInfo().get(SolverLqrDoubleParam_CostChange) = this->getLqrInfo().get(SolverLqrDoubleParam_Cost) - this->getForwPass().newCost() - 1e10*this->getForwPass().hasDiverged();
          this->getLqrInfo().get(SolverLqrDoubleParam_ExpectedCost) = std::max(-Eigen::Vector2d(this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs)[i], std::pow(this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs)[i], 2.)).dot(this->getBackPass().dV()), .0);

          if (this->getLqrInfo().get(SolverLqrDoubleParam_CostChange) > 0.0) {
            forwpass_flag_ = true;
            break;
          }
        }
      }

      // ====== STEP 4: accept (or not) the solution
      if (forwpass_flag_)
      {
        // Print status
        this->getPrinter().display(MsgLqr::STAT_success, this->getLqrInfo());

        // Decrease lambda
        if (linesearch_coeff_ > this->getLqrSetting().get(SolverLqrVectorParam_LineSearchCoeffs).minCoeff())
        {
          mult_regularization_change_ = std::min(mult_regularization_change_/this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr), 1./this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr));
          this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization)  = this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) * mult_regularization_change_ * (this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) > this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMinRegularization));
        }

        // Accept changes
        devs_flag_ = true;
        this->getLqrInfo().get(SolverLqrDoubleParam_Cost) = this->getForwPass().newCost();
        this->getOcp().stateSeq() = this->getForwPass().stateSeq();
        this->getOcp().controlSeq() = this->getForwPass().controlSeq();

        // Terminate condition
        if (std::abs(this->getLqrInfo().get(SolverLqrDoubleParam_CostChange)) < this->getLqrSetting().get(SolverLqrDoubleParam_CostChangeTolerance)) {
          this->getPrinter().display(MsgLqr::SUCC_costchange, this->getLqrInfo());
          break;
        }
      }
      else
      {
        // Increase lambda
        mult_regularization_change_ = std::max(mult_regularization_change_*this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr), this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMultRegularizationIncr));
        this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization)  = std::max(this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) * mult_regularization_change_, this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMinRegularization));

        // Print status
        this->getPrinter().display(MsgLqr::STAT_rejection, this->getLqrInfo());

        // Terminate condition
        if (this->getLqrInfo().get(SolverLqrDoubleParam_CurrentRegularization) > this->getLqrSetting().get(SolverLqrDoubleParam_BackPassMaxRegularization)) {
          this->getPrinter().display(MsgLqr::EXIT_maxlambda, this->getLqrInfo());
          break;
        }
      }
    }

    if (this->getLqrInfo().get(SolverLqrIntParam_CurrentIteration) == this->getLqrSetting().get(SolverLqrIntParam_LqrMaxIterations)) { this->getPrinter().display(MsgLqr::EXIT_maxiter, this->getLqrInfo()); }
    this->getPrinter().display(MsgLqr::STAT_summary, this->getLqrInfo());
    if (this->getLqrSetting().get(SolverLqrBoolParam_StoreData)) { this->storeSolution(); }
  }

  void SolverLqr::storeSolution()
  {
    try
    {
      int tdim=this->getOcp().tdim(), xdim=this->getOcp().xdim(), udim=this->getOcp().udim();
      Eigen::MatrixXd states(xdim, tdim+1);
      Eigen::MatrixXd control_ff(udim, tdim);

      for (int time=0; time<tdim; time++) {
        states.col(time) = this->getOcp().stateSeq().state(time).stateVector();
        control_ff.col(time) = this->getOcp().controlSeq().control(time).feedforward();
      }
      states.col(tdim) = this->getOcp().stateSeq().state(tdim).stateVector();

      YAML::Node ocp_pars;
      YAML::Node env_cfg = YAML::LoadFile(this->getLqrSetting().get(SolverLqrStringParam_ConfigFile));

      ocp_pars["solverlqr_variables"] = env_cfg["solverlqr_variables"];
      ocp_pars["solverlqr_variables"]["tdim"] = tdim;
      ocp_pars["solverlqr_variables"]["xdim"] = xdim;
      ocp_pars["solverlqr_variables"]["udim"] = udim;
      ocp_pars["solverlqr_variables"]["states"] = states;
      ocp_pars["solverlqr_variables"]["control_ff"] = control_ff;
      ocp_pars["solverlqr_variables"]["dt"] = this->getOcp().dt();
      for (int time_id=0; time_id<tdim; time_id++)
        ocp_pars["solverlqr_variables"]["control_fb_"+std::to_string(time_id)] = this->getOcp().controlSeq().control(time_id).feedback();

      std::ofstream file_out(this->getLqrSetting().get(SolverLqrStringParam_SaveLqrFile));
      file_out << ocp_pars;
    }
    catch (...)
    {
      throw std::runtime_error("Err SolverLqr (Fnc storeSolution).\n");
    }
  }

  void SolverLqr::loadSolution(const std::string& load_file)
  {
/////////////////////////////////////////////////////////////////////////////////////////
    try
    {
      YAML::Node ocp_pars = YAML::LoadFile(load_file.c_str());

      for (int time_id=0; time_id<this->getOcp().tdim(); time_id++)
        this->getOcp().controlSeq().control(time_id).feedback() = ocp_pars["solverlqr_variables"]["control_fb_"+std::to_string(time_id)].as<Eigen::MatrixXd>();

//      // loading center of mass and momentum references
//      Eigen::MatrixXd com_guess_, lmom_guess_, amom_guess_, lmomd_guess_, amomd_guess_, mat_guess_;
//      readParameter(qcqp_cfg["dynopt_params"], "time_vec", mat_guess_);
//      readParameter(qcqp_cfg["dynopt_params"], "lin_mom", lmom_guess_);
//      readParameter(qcqp_cfg["dynopt_params"], "ang_mom", amom_guess_);
//      readParameter(qcqp_cfg["dynopt_params"], "com_motion", com_guess_);
//      readParameter(qcqp_cfg["dynopt_params"], "lin_mom_rate", lmomd_guess_);
//      readParameter(qcqp_cfg["dynopt_params"], "ang_mom_rate", amomd_guess_);
//
//      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
//        dynamicsSequence().dynamicsState(time_id).centerOfMass() = Eigen::Vector3d(com_guess_.block<3,1>(0,time_id));
//        dynamicsSequence().dynamicsState(time_id).linearMomentum() = Eigen::Vector3d(lmom_guess_.block<3,1>(0,time_id));
//        dynamicsSequence().dynamicsState(time_id).angularMomentum() = Eigen::Vector3d(amom_guess_.block<3,1>(0,time_id));
//        dynamicsSequence().dynamicsState(time_id).linearMomentumRate() = Eigen::Vector3d(lmomd_guess_.block<3,1>(0,time_id));
//        dynamicsSequence().dynamicsState(time_id).angularMomentumRate() = Eigen::Vector3d(amomd_guess_.block<3,1>(0,time_id));
//      }
//
//      // loading timestep durations guess
//      dynamicsSequence().dynamicsState(0).time() = mat_guess_(0,0);
//      for (int time_id=1; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
//        dynamicsSequence().dynamicsState(time_id).time() = mat_guess_(0,time_id) - mat_guess_(0,time_id-1);
//
//      // loading vector of forces, torques and cops
//      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
//        readParameter(qcqp_cfg["dynopt_params"], "eef_frc_"+std::to_string(eff_id), mat_guess_);
//        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
//          dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id) = mat_guess_.col(time_id).head<3>();
//
//        readParameter(qcqp_cfg["dynopt_params"], "eef_trq_cp_"+std::to_string(eff_id), mat_guess_);
//        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
//          dynamicsSequence().dynamicsState(time_id).endeffectorTorqueAtContactPoint(eff_id) = mat_guess_.col(time_id).head<3>();
//
//        readParameter(qcqp_cfg["dynopt_params"], "eef_trq_"+std::to_string(eff_id), mat_guess_);
//        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
//          dynamicsSequence().dynamicsState(time_id).endeffectorTorque(eff_id).z() = mat_guess_(0, time_id);
//
//        readParameter(qcqp_cfg["dynopt_params"], "eef_cop_"+std::to_string(eff_id), mat_guess_);
//        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
//          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
//            dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).head(2) = mat_guess_.col(dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id)).head(2);
//      }
    }
    catch (std::runtime_error& e)
    {
      std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

/////////////////////////////////////////////////////////////////////////////////////////
  }

}
