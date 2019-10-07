/**
 * @file RtHQPSolver.h
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <solver/rt_optimizer/RtHQPSolver.hpp>

namespace rt_solver {

  // helper variable to define max number of ranks possible in the hierarchy
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  const int RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::num_max_ranks_;

  // constructor and destructor
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::RtHQPSolver(YAML::Node params)
    : qp_solver_(), qp_solver_interface_(qp_solver_)
  {
    reset(Max_Num_Vars);
    inequality_relaxation_ = (params["ineq_relax"] ? params["ineq_relax"].as<double>() : 1.e-6);
    eq_condition_threash_ = (params["hsol_max_eq_cond"] ? params["hsol_max_eq_cond"].as<double>() : 1.e8);
    diag_addition_for_psd_hessian_ = (params["psd_hessian_diag"] ? params["psd_hessian_diag"].as<double>() : 1.e-8);
  }

  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::~RtHQPSolver()
  {
    stopSVD2ProblemConverter();
  }

  // initialization function
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  void RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::initialize()
  {
    is_svd_computed_ = true;
    apply_ineq_slacks_ = true;

    for (int rank_id=0; rank_id<num_max_ranks_; rank_id++)
      RtMatrixUtils::resize(prev_K_[rank_id], Max_Num_Vars, 0);

    stopSVD2ProblemConverter();
    stop_svd2problem_converter_ = false;
    svd2problem_converter_thread_.reset(new boost::thread(boost::bind( &RtHQPSolver<
      All_A_Rows, Max_Num_Vars, Max_B_Rows>::doSVDComputations, this )) );
  }

  // computation of singular value decompositions
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  void RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::doSVDComputations()
  {
    while (!stop_svd2problem_converter_) {
      // wait until svd is required
      mutex_.lock();
      while (!stop_svd2problem_converter_ && is_svd_computed_) { problem2svd_converter_.wait(mutex_); }
      if (stop_svd2problem_converter_) { mutex_.unlock(); break; }

      if(B_Nprev_.rows() != 0) {
        svd_B_Nprev_.compute(B_Nprev_, Eigen::ComputeFullV);
        RtMatrixUtils::computeNullspaceMap(B_Nprev_, K_, svd_B_Nprev_, eq_condition_threash_);
        prev_K_[cur_rank_] = K_;
      }
      cur_rank_++;

      // notify main thread that svd is available now
      is_svd_computed_ = true;
      svd2problem_converter_.broadcast();
      mutex_.unlock();
    }
  }

  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  void RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::stopSVD2ProblemConverter()
  {
    if(svd2problem_converter_thread_ != NULL)
    {
      mutex_.lock();
      stop_svd2problem_converter_ = true;
      problem2svd_converter_.broadcast();
      mutex_.unlock();
      svd2problem_converter_thread_->join();
    }
  }

  // solver reset
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  void RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::reset(int num_vars)
  {
    mutex_.lock();
    cur_rank_ = 0;
    is_svd_computed_ = true;
    RtMatrixUtils::setIdentity(K_, num_vars);
    RtMatrixUtils::resize(B_Nprev_, 0, num_vars);
    mutex_.unlock();

    RtVectorUtils::resize(ahat_, 0);
    RtVectorUtils::resize(zopt_, 0);
    RtVectorUtils::resize(wopt_, 0);
    RtVectorUtils::setZero(xopt_, num_vars);
    RtMatrixUtils::resize(Ahat_, 0, num_vars);
    RtMatrixUtils::setIdentity(Nprev_, num_vars);
  }

  // main function to solve a hierarchy
  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  template<typename EqMatDer, typename EqVecDer, typename IneqMatDer, typename IneqVecDer>
  bool RtHQPSolver<All_A_Rows, Max_Num_Vars, Max_B_Rows>::solveNextTask(
    const Eigen::MatrixBase<EqMatDer>& eq_mat, const Eigen::MatrixBase<EqVecDer>& eq_vec,
    const Eigen::MatrixBase<IneqMatDer>& ineq_mat, const Eigen::MatrixBase<IneqVecDer>& ineq_vec)
  {
    #ifndef RTEIG_NO_ASSERTS
      assert(eq_mat.rows() <= Max_B_Rows && "HierarchicalTask exceeds max number of equality constraints!");
      assert(eq_mat.rows() == 0 || (eq_mat.cols() == numVars() && "HierarchicalTask has wrong number of columns!"));
      assert(ineq_mat.rows() == 0 || (ineq_mat.cols() == numVars() && "HierarchicalTask has wrong number of columns!"));
      assert(Ahat_.rows() + ineq_mat.rows() <= All_A_Rows && "HierarchicalTask exceeds max number of inequality constraints!");
    #endif

    if (eq_mat.rows() == 0 && ineq_mat.rows() == 0) { return true; }

    // wait until svd is computed
    mutex_.lock();

    // use svd
    if (K_.cols() == 0 && ineq_mat.rows() == 0) { mutex_.unlock(); return true; }

    //compute nullspace mapping of previous B_Nprev_
    if(Nprev_.cols() == K_.rows()) { Nprev_ *= K_; }
    const int dim_z = Nprev_.cols();
    const int dim_w = ineq_mat.rows();

    /*
     * First Set of Updates
     */

    //update kernel_to_ineqs vector with previous results
    if (ahat_.rows() != 0)
    {
      #ifndef RTEIG_NO_ASSERTS
        if (Ahat_.cols() != zopt_.rows()) { assert(Ahat_.cols() == zopt_.rows()); }
      #endif
      ahat_ += Ahat_ * zopt_;
      for (int id=0; id<ahat_.size(); id++)
        if(ahat_[id] >= -inequality_relaxation_)
          ahat_[id] = -inequality_relaxation_;

      if (wopt_.rows() != 0) //previous task had inequality constraints
        ahat_.bottomRows(wopt_.rows()) += wopt_;
    }

    /*
     * Second Set of Updates
     */

    //update kernel_to_ineqs vector with new inequalities
    if (ineq_mat.rows() != 0) { RtVectorUtils::append(ahat_, ineq_mat * xopt_ + ineq_vec); }

    //update kernel_to_ineqs matrix with previous results
    if(Ahat_.cols() == K_.rows()) { Ahat_ *= K_; }

    //update kernel_to_ineqs mapping with new inequalities
    if (ineq_mat.rows() != 0) { RtMatrixUtils::append(Ahat_, ineq_mat * Nprev_); }

    /*
     * Construct and solve the problem
     */

    // reset solver
    if (apply_ineq_slacks_) { qp_solver_interface_.reset(dim_z + dim_w, 0, Ahat_.rows()); }
    else                    { qp_solver_interface_.reset(dim_z, 0, Ahat_.rows()); }
    qp_solver_interface_.qp_properties_ |= QpSolverInterface::ePP_MightZeroIneqs;


    if (eq_mat.rows() != 0)
    {
      B_Nprev_ = eq_mat * Nprev_;
      RtQuadraticUtils::addFromNorm(qp_solver_interface_.objectiveQuadPart().topLeftCorner(dim_z, dim_z),
                                    qp_solver_interface_.objectiveLinPart().topRows(dim_z), B_Nprev_,
                                    eq_vec + eq_mat * xopt_);
    }

    // notify problem2svd_converter that it can proceed
    is_svd_computed_ = false;
    problem2svd_converter_.broadcast();
    mutex_.unlock();


    if (dim_w != 0 && apply_ineq_slacks_) { qp_solver_interface_.objectiveQuadPart().block(dim_z, dim_z, dim_w, dim_w).setIdentity(); }

    // add slack on diagonal of hessian to make it positive definite
    qp_solver_interface_.objectiveQuadPart().diagonal().array() += diag_addition_for_psd_hessian_;

    // setup inequalities
    if (Ahat_.rows() != 0)
    {
      if(apply_ineq_slacks_) {
        typename RtMatrix<All_A_Rows, Max_Num_Vars + All_A_Rows>::d ineq_mat_slack;
        RtMatrixUtils::setZero(ineq_mat_slack, Ahat_.rows(), dim_z + dim_w);
        ineq_mat_slack.leftCols(dim_z) = Ahat_;
        ineq_mat_slack.bottomRightCorner(dim_w, dim_w).setIdentity();
        RtAffineUtils::appendAtColumn(qp_solver_interface_.ineqConstraintsMat(), qp_solver_interface_.ineqConstraintsVec(), ineq_mat_slack, ahat_, 0);
      } else {
        RtAffineUtils::appendAtColumn(qp_solver_interface_.ineqConstraintsMat(), qp_solver_interface_.ineqConstraintsVec(), Ahat_, ahat_, 0);
      }
    }

    // solve
    if(apply_ineq_slacks_)
    {
      boost::posix_time::ptime wait_start(boost::posix_time::microsec_clock::local_time());
      try { qp_solver_interface_.optimize(); }
      catch (...) { std::cout << "Qp solver failed" << std::endl; }

      boost::posix_time::ptime wait_end(boost::posix_time::microsec_clock::local_time());
      boost::posix_time::time_duration dur = wait_end - wait_start;
      qpsolve_duration_ = dur.total_microseconds()/1000000.0;

      #ifndef RTEIG_NO_ASSERTS
        assert(((!qp_solver_interface_.isOptimized()) || (qp_solver_interface_.isOptimized() && qp_solver_interface_.checkSolution(false))) && "Hierarchical task could not be solved");
      #endif
    }

    // synchronize with svd computation
    boost::posix_time::ptime wait_start(boost::posix_time::microsec_clock::local_time());
    mutex_.lock();
    while (!is_svd_computed_) { svd2problem_converter_.wait(mutex_); }
    mutex_.unlock();

    boost::posix_time::ptime wait_end(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration dur = wait_end - wait_start;
    wait_duration_ = dur.total_microseconds()/1000000.0;

    /*
     * Update the solution
     */
    if (!apply_ineq_slacks_) { zopt_ = -eq_mat.transpose()*(eq_mat*eq_mat.transpose()).inverse()*eq_vec; }
    else {
      if (qp_solver_interface_.isOptimized()) { zopt_ = qp_solver_interface_.solution().topRows(dim_z); }
      else                                    { RtVectorUtils::setZero(zopt_, dim_z); }
    }
    xopt_ += Nprev_ * zopt_;
    if (dim_w != 0) {
      if(apply_ineq_slacks_ && qp_solver_interface_.isOptimized()) {
        RtVectorUtils::resize(wopt_, dim_w);
        wopt_ = qp_solver_interface_.solution().segment(dim_z, dim_w);
      } else {
        RtVectorUtils::setZero(wopt_, dim_w);
      }
    }

    return qp_solver_interface_.isOptimized() || !apply_ineq_slacks_;
  }

}
