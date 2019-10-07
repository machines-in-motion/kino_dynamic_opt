/**
 * @file RtHQPSolver.hpp
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <solver/rt_optimizer/RtMutex.hpp>
#include <solver/rt_optimizer/RtQPSolver.hpp>

namespace rt_solver {

  template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  class RtHQPSolver
  {
    private:
      static const int num_max_ranks_=7;
      typedef RtQPSolver<Max_Num_Vars+All_A_Rows, 0, All_A_Rows> QpSolver;
      typedef RtQPSolverInterface<Max_Num_Vars + All_A_Rows, 0, All_A_Rows> QpSolverInterface;

    public:
      // constructor and destructor
      RtHQPSolver(YAML::Node params);
      virtual ~RtHQPSolver();

      // functions to interface the solver behavior
      void initialize();
      void reset(int num_vars);
      template<typename EqMatDer, typename EqVecDer, typename IneqMatDer, typename IneqVecDer>
      bool solveNextTask(const Eigen::MatrixBase<EqMatDer>& eq_mat, const Eigen::MatrixBase<EqVecDer>& eq_vec,
                         const Eigen::MatrixBase<IneqMatDer>& ineq_mat, const Eigen::MatrixBase<IneqVecDer>& ineq_vec);

      // helper getter and setter methods
      bool& applyIneqSlacks() { return apply_ineq_slacks_; }
      const double& qpWaitDuration() const { return wait_duration_; }
      const bool& applyIneqSlacks() const { return apply_ineq_slacks_; }
      const double& qpSolveDuration() const { return qpsolve_duration_; }

      int nullspaceDimension() const { return K_.cols(); }
      typename RtVector<Max_Num_Vars>::d& solution() { return xopt_; }
      const typename RtVector<Max_Num_Vars>::d& solution() const { return xopt_; }

    private:
      // helper internal functions
      int numVars() const { return xopt_.size(); }
      const typename RtMatrix<Max_Num_Vars, Max_Num_Vars>::d contrainedEqualityMat() const { return Nprev_; };

      // helper functions to handle svd operations
      void doSVDComputations();
      void stopSVD2ProblemConverter();


    private:
      // helper integer variables
      int cur_rank_;

      // helper boolean variables
      bool apply_ineq_slacks_, is_svd_computed_, stop_svd2problem_converter_;

      // helper double variables
      double wait_duration_, qpsolve_duration_, eq_condition_threash_,
             inequality_relaxation_, diag_addition_for_psd_hessian_;

      // instances of problem optimizer and its interface
      QpSolver qp_solver_;
      QpSolverInterface& qp_solver_interface_;

      // rt elements
      typename RtVector<All_A_Rows>::d ahat_;
      typename RtVector<All_A_Rows>::d wopt_;
      typename RtVector<Max_Num_Vars>::d xopt_;
      typename RtVector<Max_Num_Vars>::d zopt_;
      typename RtMatrix<Max_Num_Vars, Max_Num_Vars>::d K_;
      typename RtMatrix<All_A_Rows, Max_Num_Vars>::d Ahat_;
      typename RtMatrix<Max_B_Rows, Max_Num_Vars>::d B_Nprev_;
      typename RtMatrix<Max_Num_Vars, Max_Num_Vars>::d Nprev_;
      typename RtMatrix<Max_Num_Vars, Max_Num_Vars>::d prev_K_[num_max_ranks_];
      Eigen::JacobiSVD<typename RtMatrix<Max_B_Rows, Max_Num_Vars>::d> svd_B_Nprev_;

      // rt communication and coordination
      RtMutex mutex_;
      RtCond svd2problem_converter_, problem2svd_converter_;
      boost::shared_ptr<boost::thread> svd2problem_converter_thread_;

  };

}
