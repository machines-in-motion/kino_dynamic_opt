/**
 * @file RtQPSolverInterface.hpp
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <vector>
#include <iostream>
#include <solver/rt_optimizer/RtMatrix.hpp>

namespace rt_solver {

  template <int max_dim_qp, int max_num_eq, int max_num_ineq>
  class RtQPSolverInterface
  {
    protected:
      typename RtVector<max_dim_qp>::d g_;
      typename RtVector<max_dim_qp>::d sol_;
      typename RtVector<max_num_eq>::d eq_vec_;
      typename RtVector<max_num_ineq>::d ineq_vec_;
      typename RtMatrix<max_dim_qp, max_dim_qp>::d H_;
      typename RtMatrix<max_num_eq, max_dim_qp>::d Eq_mat_;
      typename RtMatrix<max_num_ineq, max_dim_qp>::d Ineq_mat_;

    public:
      enum QPProperties {
        //ePP_ObjectivePD = 1 << 3,
        //ePP_ObjectivePD = 1 << 1,
        //ePP_MightZeroIneqs = 1 << 2
        ePP_MightZeroIneqs = 1 << 0
      };

      int qp_properties_;

      // getter and setter methods
      typename RtVector<max_dim_qp>::d& solution() { return sol_; }
      const typename RtVector<max_dim_qp>::d& solution() const { return sol_; }

      typename RtMatrix<max_dim_qp, max_dim_qp>::d& objectiveQuadPart() { return H_; }
      const typename RtMatrix<max_dim_qp, max_dim_qp>::d& objectiveQuadPart() const { return H_; }

      typename RtVector<max_dim_qp>::d& objectiveLinPart() { return g_; }
      const typename RtVector<max_dim_qp>::d& objectiveLinPart() const { return g_; }

      typename RtMatrix<max_num_eq, max_dim_qp>::d& eqConstraintsMat() { return Eq_mat_; }
      const typename RtMatrix<max_num_eq, max_dim_qp>::d& eqConstraintsMat() const { return Eq_mat_; }

      typename RtVector<max_num_eq>::d& eqConstraintsVec() { return eq_vec_; }
      const typename RtVector<max_num_eq>::d& eqConstraintsVec() const { return eq_vec_; }

      typename RtMatrix<max_num_ineq, max_dim_qp>::d& ineqConstraintsMat() { return Ineq_mat_; }
      const typename RtMatrix<max_num_ineq, max_dim_qp>::d& ineqConstraintsMat() const { return Ineq_mat_; }

      typename RtVector<max_num_ineq>::d& ineqConstraintsVec() { return ineq_vec_; }
      const typename RtVector<max_num_ineq>::d& ineqConstraintsVec() const { return ineq_vec_; }

      int numVariables() const { return H_.rows(); }
      int numEqConstr() const { return Eq_mat_.rows(); }
      int numIneqConstr() const { return Ineq_mat_.rows(); }

      // constructor and destructor
      RtQPSolverInterface() { this->reset(); }
      virtual ~RtQPSolverInterface(){}

      // functions to be implemented to use the interface
      bool virtual optimize() =0;
      bool virtual isOptimized() const =0;

      // Append equality constraints mat*x + vec = 0
      template <typename Derived, typename OtherDerived>
      inline void appendEqualities(const Eigen::MatrixBase<Derived>& mat, const Eigen::MatrixBase<OtherDerived>& vec)
      {
        RtAffineUtils::append(Eq_mat_, eq_vec_, mat, vec);
      }

      template <typename Derived>
      inline void appendEqualities(const Eigen::MatrixBase<Derived>& mat)
      {
        RtAffineUtils::append(Eq_mat_, eq_vec_, mat);
      }

      // Append inequality constraints mat*x + vec <= 0
      template <typename Derived, typename OtherDerived>
      inline void appendInequalities(const Eigen::MatrixBase<Derived>& mat, const Eigen::MatrixBase<OtherDerived>& vec)
      {
        RtAffineUtils::append(Ineq_mat_, ineq_vec_, mat, vec);
      }

      template <typename Derived>
      inline void appendInequalities(const Eigen::MatrixBase<Derived>& mat)
      {
        RtAffineUtils::append(Ineq_mat_, ineq_vec_, mat);
      }

      /*
       * Helper functions for the interface
       */

      // reset solver
      inline virtual void reset()
      {
        qp_properties_ = 0;
        g_ = Eigen::Matrix<double, max_dim_qp, 1>::Zero();
        sol_ = Eigen::Matrix<double, max_dim_qp, 1>::Zero();
        H_ = Eigen::Matrix<double, max_dim_qp, max_dim_qp>::Zero();

        RtVectorUtils::setZero(eq_vec_, 0);
        RtVectorUtils::setZero(ineq_vec_, 0);
        RtMatrixUtils::setZero(Eq_mat_, 0, max_dim_qp);
        RtMatrixUtils::setZero(Ineq_mat_, 0, max_dim_qp);
      }

      inline virtual void reset(int dim_qp, int /*num_eq*/, int /*num_ineq*/)
      {
        qp_properties_ = 0;
        RtVectorUtils::setZero(g_, dim_qp);
        RtVectorUtils::setZero(eq_vec_, 0);
        RtVectorUtils::setZero(ineq_vec_, 0);
        RtVectorUtils::setZero(sol_, dim_qp);
        RtMatrixUtils::setZero(H_, dim_qp, dim_qp);
        RtMatrixUtils::setZero(Eq_mat_, 0, dim_qp);
        RtMatrixUtils::setZero(Ineq_mat_, 0, dim_qp);
      }

      // compute optimality condition
      double computeOptimalityCondition() const
      {
        typename RtMatrix<max_dim_qp, max_num_eq+max_num_ineq>::d constr_jac_transp;
        constr_jac_transp.resize(numVariables(), numEqConstr()+numIneqConstr());
        constr_jac_transp.leftCols(numEqConstr()) = eqConstraintsMat().transpose();

        double eq_vio_sqr = 0;
        if (numEqConstr() != 0) { eq_vio_sqr = (eqConstraintsMat()*solution() + eqConstraintsVec()).squaredNorm(); }

        int to_col=numEqConstr();
        double ineq_vio_sqr = 0;
        for (int r=0; r<numIneqConstr(); r++) {
          const double ineq_val = ineqConstraintsMat().row(r).dot(solution()) + ineqConstraintsVec()[r];
          const double vio = std::max(0., ineq_val);
          ineq_vio_sqr += vio*vio;
          if (ineq_val > -1e-8) {
            constr_jac_transp.col(to_col) = ineqConstraintsMat().row(r).transpose();
            ++to_col;
          }
        }

        constr_jac_transp.conservativeResize(constr_jac_transp.rows(), to_col);
        typename RtVector<max_dim_qp>::d obj_grad;
        obj_grad.resize(numVariables());
        obj_grad = objectiveQuadPart()*solution() + objectiveLinPart();
        double grad_sqr_norm;
        if (constr_jac_transp.cols() != 0) {
          typename RtVector<max_num_eq+max_num_ineq>::d lagr;
          lagr.resize(constr_jac_transp.cols());
          lagr = constr_jac_transp.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(obj_grad);
          grad_sqr_norm = (constr_jac_transp*lagr-obj_grad).squaredNorm();
        } else {
          grad_sqr_norm = obj_grad.squaredNorm();
        }
        return std::sqrt(grad_sqr_norm + ineq_vio_sqr + eq_vio_sqr);
      }

      // check solution
      virtual bool checkSolution(bool print = true, double eq_threashold = 0.001, double ineq_threashold = 0.001)
      {
        const double eq_norm = (eqConstraintsMat() * solution() + eqConstraintsVec()).norm();
        typename RtVector<max_num_ineq>::d ineq_dist = ineqConstraintsMat() * solution() + ineqConstraintsVec();
        for (int i=0; i<numIneqConstr(); ++i)
          ineq_dist[i] *= ineq_dist[i] <= 0.0 ? 0.0 : 1.0;
        const double ineq_norm = (ineq_dist).norm();
        const double lagr_grad_norm = -1;//compute_optimality_condition();
        bool is_sol_valid = eq_norm <= eq_threashold && ineq_norm <= ineq_threashold;

        if (print || (!is_sol_valid && isOptimized()))
        {
          std::cout << "qp check (num vars, num equalities, num inequalities, objective value): " <<
                       numVariables() << " " << numEqConstr() << " " << numIneqConstr() << " " <<
                       solution().transpose() * objectiveQuadPart() * solution() + 2*objectiveLinPart().transpose()*solution()<< std::endl;
          std::cout << "  lagrangian gradient norm:   " << lagr_grad_norm << std::endl;
          std::cout << "  equality squared dist:   " << eq_norm << std::endl;
          std::cout << "  inequality squared dist: " << ineq_norm << std::endl;
          std::cout << "  isOptimized(): " << ( isOptimized()? "yes" : "no" ) << std::endl;
        }
        return is_sol_valid;
      }

      // evaluate the problem condition number
      void printProblemCondition(std::ostream& stream)
      {
        stream << "QP:" << std::endl;
        stream << "  num Variables: " << numVariables() << std::endl;
        stream << "  num Equalities: " << numEqConstr() << std::endl;
        stream << "  num Inequalities: " << numIneqConstr() << std::endl;

        Eigen::JacobiSVD<typename RtMatrix<max_dim_qp, max_dim_qp>::d > Hess_svd;
        if (objectiveQuadPart().rows() > 0)
        {
          Hess_svd.compute(objectiveQuadPart());
          stream << "  Hessian condition: " << Hess_svd.singularValues().maxCoeff()/
                    Hess_svd.singularValues().minCoeff() << " = " << Hess_svd.
                    singularValues().maxCoeff() << " / " << Hess_svd.singularValues().minCoeff() << std::endl;
        }
        Eigen::JacobiSVD<typename RtMatrix<max_num_eq, max_dim_qp>::d > EqConstr_svd;
        if (eqConstraintsMat().rows() > 0)
        {
          EqConstr_svd.compute(eqConstraintsMat());
          stream << "  Equality Matrix condition: " << EqConstr_svd.singularValues().maxCoeff()/
                    EqConstr_svd.singularValues().minCoeff() << " = " << EqConstr_svd.
                    singularValues().maxCoeff() << " / " << EqConstr_svd.singularValues().minCoeff() << std::endl;
        }
        Eigen::JacobiSVD<typename RtMatrix<max_num_ineq, max_dim_qp>::d > IneqConstr_svd;
        if (ineqConstraintsMat().rows() > 0)
        {
          IneqConstr_svd.compute(ineqConstraintsMat());
          stream << "  Inequality Matrix condition: " << IneqConstr_svd.singularValues().maxCoeff()/
                    IneqConstr_svd.singularValues().minCoeff() << " = " << IneqConstr_svd.
                    singularValues().maxCoeff() << " / " << IneqConstr_svd.singularValues().minCoeff() << std::endl;
        }

        if(numVariables() < 10)
        {
          stream << "  Hessian: " << objectiveQuadPart() << std::endl;
          stream << "  linear part: " << objectiveLinPart().transpose() << std::endl;
          stream << "  Equality Matrix: " << eqConstraintsMat() << std::endl;
          stream << "  Equality Vector: " << eqConstraintsVec().transpose() << std::endl;
          stream << "  Inequality Matrix: " << ineqConstraintsMat() << std::endl;
          stream << "  Inequality Vector: " << ineqConstraintsVec().transpose() << std::endl;
        }
      }
  };

}
