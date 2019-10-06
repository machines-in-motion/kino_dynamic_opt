/*
 * Copyright [2019] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <solver/rt_optimizer/RtQPSolverInterface.hpp>

namespace rt_solver {

  template<int nVars, int nEqCon, int nIneqCon>
  class RtQPSolver : public RtQPSolverInterface<nVars, nEqCon, nIneqCon>
  {
    public:
      // default constructor and destructor methods
      RtQPSolver(): solver_return_(std::numeric_limits<double>::infinity())
      {
        cleanup_ineqs_ = false;
        is_inverse_provided_ = false;
      }
      virtual ~RtQPSolver(){}

      // function to optimize problem and return a flag about the exit code
      bool optimize()
      {
        if (BaseClass::objectiveQuadPart().rows() == 0) { return true; }
        solver_return_ = solveQP(BaseClass::objectiveQuadPart(),
          BaseClass::objectiveLinPart(), BaseClass::eqConstraintsMat(),
          BaseClass::eqConstraintsVec(), BaseClass::ineqConstraintsMat(),
          BaseClass::ineqConstraintsVec(), BaseClass::solution());

        return isOptimized();
      }
      bool isOptimized() const { return solver_return_ != std::numeric_limits<double>::infinity(); }

    private:
      // definition of base class
      typedef RtQPSolverInterface<nVars, nEqCon, nIneqCon> BaseClass;

      // internal helper reset functions
      inline void reset()
      {
        BaseClass::reset();
        RtMatrixUtils::setZero(Hessian_factor_inv_, nVars, nVars);
        solver_return_ = std::numeric_limits<double>::infinity();
      }
      inline void reset(int dim_qp, int num_eq, int num_ineq)
      {
        BaseClass::reset(dim_qp, num_eq, num_ineq);
        RtMatrixUtils::setZero(Hessian_factor_inv_, dim_qp, dim_qp);
        solver_return_ = std::numeric_limits<double>::infinity();
      }

      // helper optimization variables
      bool cleanup_ineqs_;
      double solver_return_;
      bool is_inverse_provided_;
      typename RtMatrix<nVars,nVars>::d Hessian_factor_inv_;
      Eigen::LLT<typename RtMatrix<nVars,nVars>::d,Eigen::Lower> chol_;

      // helper function to measure a distance
      template<typename Scalar>
      inline Scalar distance(Scalar a, Scalar b)
      {
        Scalar a1, b1, t;
        a1 = std::abs(a);
        b1 = std::abs(b);
        if (a1 > b1) {
          t = (b1 / a1);
          return a1 * std::sqrt(1.0 + t * t);
        } else if (b1 > a1) {
          t = (a1 / b1);
          return b1 * std::sqrt(1.0 + t * t);
        }
        return a1 * std::sqrt(2.0);
      }

      // helper functions to update terms
      inline void computeD(typename RtVector<nVars>::d & d,
        const typename RtMatrix<nVars,nVars>::d & J, const typename RtVector<nVars>::d & np)
      { d = J.adjoint() * np; }

      inline void updateZ(typename RtVector<nVars>::d & z,
        const typename RtMatrix<nVars,nVars>::d & J,
        const typename RtVector<nVars>::d & d, int iq)
      { z = J.rightCols(J.cols()-iq) * d.tail(J.cols()-iq); }

      inline void updateR(const typename RtMatrix<nVars,nVars>::d & R,
        typename RtVector<nIneqCon+nEqCon>::d& r, const typename RtVector<nVars>::d& d, int iq)
      { r.head(iq)= R.topLeftCorner(iq,iq).template triangularView<Eigen::Upper>().solve(d.head(iq)); }

      // helper functions to add and delete constraints
      inline bool addConstraint(
        typename RtMatrix<nVars,nVars>::d & R, typename RtMatrix<nVars,nVars>::d & J,
        typename RtVector<nVars>::d & d, int& iq, double& R_norm)
      {
        int n=J.rows();

        int j, k;
        double cc, ss, h, t1, t2, xny;

        // we have to find the Givens rotation which will reduce the element d(j) to zero.
        // if it is already zero we don't have to do anything, except of decreasing j */
        for (j=n-1; j>=iq+1; j--)
        {
          // The Givens rotation is done with the matrix (cc cs, cs -cc). If cc is one, then
          // element (j) of d is zero compared with element (j-1). Hence we don't have to do
          // anything. If cc is zero, then we just have to switch column (j) and column (j-1)
          // of J. Since we only switch columns in J, we have to be careful how we update d
          // depending on the sign of gs. Otherwise we have to apply the Givens rotation to
          // these columns. The i-1 element of d has to be updated to h.

          cc = d(j - 1);
          ss = d(j);
          h = distance(cc, ss);
          if (h == 0.0)
            continue;

          d(j) = 0.0;
          ss = ss / h;
          cc = cc / h;
          if (cc < 0.0) {
            cc = -cc;
            ss = -ss;
            d(j - 1) = -h;
          } else {
            d(j - 1) = h;
          }
          xny = ss / (1.0 + cc);
          for (k = 0; k < n; k++)
          {
            t1 = J(k,j - 1);
            t2 = J(k,j);
            J(k,j - 1) = t1 * cc + t2 * ss;
            J(k,j) = xny * (t1 + J(k,j - 1)) - t2;
          }
        }

        // update the number of constraints added
        iq++;

        // To update R we have to put the iq components of the d vector into column iq - 1 of R
        R.col(iq-1).head(iq) = d.head(iq);
        if (std::abs(d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm)
      	  return false; // problem degenerate

        R_norm = std::max<double>(R_norm, std::abs(d(iq - 1)));
        return true;
      }

      inline void deleteConstraint(
        typename RtMatrix<nVars,nVars>::d& R, typename RtMatrix<nVars,nVars>::d& J,
        typename RtVector<nIneqCon+nEqCon>::i & A, typename RtVector<nIneqCon+nEqCon>::d & u, int p, int& iq, int l)
      {
          int n = J.rows();

          int i, j, k;
          int qq =0;
          double cc, ss, h, xny, t1, t2;

          // Find the index qq for active constraint l to be removed
          for (i = p; i < iq; i++)
            if (A(i) == l) { qq = i; break; }

          // remove the constraint from the active set and the duals
          for (i = qq; i < iq - 1; i++) {
            A(i) = A(i + 1);
            u(i) = u(i + 1);
            R.col(i) = R.col(i+1);
          }

          A(iq - 1) = A(iq);
          u(iq - 1) = u(iq);
          A(iq) = 0;
          u(iq) = 0.0;
          for (j=0; j<iq; j++)
            R(j,iq - 1) = 0.0;

          // constraint has been fully removed
          iq--;

          if (iq == 0) { return; }

          for (j = qq; j < iq; j++) {
            cc = R(j,j);
            ss = R(j + 1,j);
            h = distance(cc, ss);
            if (h == 0.0) { continue; }
            cc = cc / h;
            ss = ss / h;
            R(j + 1,j) = 0.0;
            if (cc < 0.0) {
              R(j,j) = -h;
              cc = -cc;
              ss = -ss;
              } else { R(j,j) = h; }

              xny = ss / (1.0 + cc);
              for (k = j + 1; k < iq; k++) {
                t1 = R(j,k);
                t2 = R(j + 1,k);
                R(j,k) = t1 * cc + t2 * ss;
                R(j + 1,k) = xny * (t1 + R(j,k)) - t2;
              }
              for (k = 0; k < n; k++) {
                t1 = J(k,j);
                t2 = J(k,j + 1);
                J(k,j) = t1 * cc + t2 * ss;
                J(k,j + 1) = xny * (J(k,j) + t1) - t2;
              }
            }
        }

      /**
       * Main function to solve QP using EigQuadProg
       * min.  x' Hess x + 2 g0' x
       * s.t.  CE x + ce0 = 0
       *       CI x + ci0 <= 0
       */
      inline double solveQP(
        const typename RtMatrix<nVars,nVars>::d & Hess, const typename RtVector<nVars>::d & g0,
        const typename RtMatrix<nEqCon, nVars>::d & CE, const typename RtVector<nEqCon>::d & ce0,
        const typename RtMatrix<nIneqCon, nVars>::d & CI, const typename RtVector<nIneqCon>::d & ci0,
        typename RtVector<nVars>::d & x)
      {
        int i, k, l;
        int ip, me, mi;
        int n=Hess.rows();
        int p=ce0.size();
        int m=ci0.size();
        typename RtMatrix<nVars,nVars>::d R;

        typename RtVector<nIneqCon+nEqCon>::d s, r, u;
        RtVectorUtils::resize(s, p+m);
        RtVectorUtils::resize(r, p+m);
        RtVectorUtils::resize(u, p+m);
        typename RtVector<nVars+nEqCon>::d u_old;
        RtVectorUtils::resize(u_old, p+n);
        typename RtVector<nVars>::d z, d, np, x_old;
        RtVectorUtils::resize(z, n);
        RtVectorUtils::resize(d, n);
        RtVectorUtils::resize(np, n);
        RtVectorUtils::resize(x_old, n);
        double f_value, psi, c1, c2, sum, ss, R_norm;
        const double inf = std::numeric_limits<double>::infinity();
        double t, t1, t2; /* t is the step length, which is the minimum of the partial step length t1
                           * and the full step length t2 */
        typename RtVector<nIneqCon+nEqCon>::i A, A_old, iai, iaexcl;
        RtVectorUtils::resize(A, p+m);
        RtVectorUtils::resize(A_old, p+m);
        RtVectorUtils::resize(iai, p+m);
        RtVectorUtils::resize(iaexcl, p+m);
        // int q; // warning unusued
        int iq, iter = 0;

          me = p; /* number of equality constraints */
          mi = m; /* number of inequality constraints */
          // q = 0;  /* size of the active set A (containing the indices of the active constraints) */

          /*
           * Preprocessing phase
           */
          /* compute the trace of the original matrix Hess */
          c1 = Hess.trace();

          /* decompose the matrix Hess in the form LL^T */
          if(!is_inverse_provided_) { chol_.compute(Hess); }

          /* initialize the matrix R */
          RtVectorUtils::setZero(d, n);
          RtMatrixUtils::setZero(R, n, n);
          R_norm = 1.0; /* this variable will hold the norm of the matrix R */

          /* compute the inverse of the factorized matrix Hess^-1, this is the initial value for H */
          // Hessian_factor_inv_ = L^-T
          if(!is_inverse_provided_) {
            RtMatrixUtils::setIdentity(Hessian_factor_inv_, n);
            Hessian_factor_inv_ = chol_.matrixU().solve(Hessian_factor_inv_);

            #ifndef RTEIG_NO_ASSERTS
              assert(Hessian_factor_inv_.rows() == n && Hessian_factor_inv_.cols() == n);
            #endif
          } else {
            #ifndef RTEIG_NO_ASSERTS
              //std::cout << "hess*inv norm: " << (Hess*Hessian_factor_inv_*Hessian_factor_inv_.transpose() - Eigen::MatrixXd::Identity(Hess.rows(), Hess.cols())) << std::endl;
              assert((Hess*Hessian_factor_inv_*Hessian_factor_inv_.transpose() - Eigen::MatrixXd::Identity(Hess.rows(), Hess.cols())).norm() < 0.0001 && "inverse is weird");
            #endif
          }
          c2 = Hessian_factor_inv_.trace();

          /* c1 * c2 is an estimate for cond(Hess) */

          /*
           * Find the unconstrained minimizer of the quadratic form 0.5 * x Hess x + g0 x
           * this is a feasible point in the dual space
           * x = Hess^-1 * g0
           */
          if(is_inverse_provided_) { x = Hessian_factor_inv_*Hessian_factor_inv_.transpose()*g0; }
          else                     { x = chol_.solve(g0); }
          x = -x;

          /* and compute the current solution value */
          f_value = 0.5 * g0.dot(x);

          /* Add equality constraints to the working set A */
          iq = 0;
          for (i = 0; i < me; i++)
          {
            np = CE.row(i);
            computeD(d, Hessian_factor_inv_, np);
            updateZ(z, Hessian_factor_inv_, d,  iq);
            updateR(R, r, d,  iq);

            /* compute full step length t2: i.e., the minimum step in primal space s.t. the contraint
               becomes feasible */
            t2 = 0.0;
            if (std::abs(z.dot(z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
              t2 = (-np.dot(x) - ce0(i)) / z.dot(np);

            x += t2 * z;

            /* set u = u+ */
            u(iq) = t2;
            u.head(iq) -= t2 * r.head(iq);

            /* compute the new solution value */
            f_value += 0.5 * (t2 * t2) * z.dot(np);
            A(i) = -i - 1;

            if (!addConstraint(R, Hessian_factor_inv_, d, iq, R_norm))
            {
              // FIXME: it should raise an error
              // Equality constraints are linearly dependent
              #ifndef RTEIG_NO_ASSERTS
                assert(false && "equality constraints are linearly dependent");
              #endif
              return f_value;
            }
          }

          /* set iai = K \ A */
          for (i = 0; i < mi; i++)
            iai(i) = i;

          l1:  iter++;

          /* step 1: choose a violated constraint */
          for (i = me; i < iq; i++)
          {
            ip = A(i);
            iai(ip) = -1;
          }

          /* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
          ss = 0.0;
          psi = 0.0; /* this value will contain the sum of all infeasibilities */
          ip = 0; /* ip will be the index of the chosen violated constraint */
          for (i = 0; i < mi; i++)
          {
             iaexcl(i) = 1;
             sum = -(CI.row(i).dot(x) + ci0(i));
             s(i) = sum;
             psi += std::min(0.0, sum);
          }

          if (std::abs(psi) <= mi * std::numeric_limits<double>::epsilon() * c1 * c2* 100.0)
          {
            /* numerically there are not infeasibilities anymore */
            // q = iq;
            return f_value;
          }

          /* save old values for u, x and A */
           u_old.head(iq) = u.head(iq);
           A_old.head(iq) = A.head(iq);
           x_old = x;

          l2: /* Step 2: check for feasibility and determine a new S-pair */
          for (i = 0; i < mi; i++)
          {
            if (s(i) < ss && iai(i) != -1 && iaexcl(i))
            {
              ss = s(i);
              ip = i;
            }
          }
          if (ss >= 0.0)
          {
            // q = iq;
            return f_value;
          }

          /* set np = n(ip) */
          np = -CI.row(ip);

          /* set u = (u 0)^T */
          u(iq) = 0.0;

          /* add ip to the active set A */
          A(iq) = ip;

          l2a: /* Step 2a: determine step direction */
          /* compute z = H np: the step direction in the primal space (through Hessian_factor_inv_, see the paper) */
          computeD(d, Hessian_factor_inv_, np);
          //updateZ(z, Hessian_factor_inv_, d, iq);
          if(iq >= Hessian_factor_inv_.cols())
          {
            //throw std::runtime_error("iq >= Hessian_factor_inv_.cols()");
            z.setZero();
          } else {
            updateZ(z, Hessian_factor_inv_, d, iq);
          }
          /* compute N* np (if q > 0): the negative of the step direction in the dual space */
          updateR(R, r, d, iq);

          /* Step 2b: compute step length */
          l = 0;
          /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
          t1 = inf; /* +inf */
          /* find the index l s.t. it reaches the minimum of u+(x) / r */
          for (k = me; k < iq; k++)
          {
            double tmp;
            if (r(k) > 0.0 && ((tmp = u(k) / r(k)) < t1) )
            {
              t1 = tmp;
              l = A(k);
            }
          }
          /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
          if (std::abs(z.dot(z))  > std::numeric_limits<double>::epsilon()) // i.e. z != 0
            t2 = -s(ip) / z.dot(np);
          else
            t2 = inf; /* +inf */

          /* the step is chosen as the minimum of t1 and t2 */
          t = std::min(t1, t2);

          /* Step 2c: determine new S-pair and take step: */

          /* case (i): no step in primal or dual space */
          if (t >= inf)
          {
            /* QPP is infeasible */
            // FIXME: unbounded to raise
            // q = iq;
            return inf;
          }
          /* case (ii): step in dual space */
          if (t2 >= inf)
          {
            /* set u = u +  t * [-r 1) and drop constraint l from the active set A */
            u.head(iq) -= t * r.head(iq);
            u(iq) += t;
            iai(l) = l;
            deleteConstraint(R, Hessian_factor_inv_, A, u, p, iq, l);

            goto l2a;
          }

          /* case (iii): step in primal and dual space */

          x += t * z;
          /* update the solution value */
          f_value += t * z.dot(np) * (0.5 * t + u(iq));

          u.head(iq) -= t * r.head(iq);
          u(iq) += t;

          if (t == t2)
          {
            /* full step has taken */
            /* add constraint ip to the active set*/
            if (!addConstraint(R, Hessian_factor_inv_, d, iq, R_norm))
            {
              iaexcl(ip) = 0;
              deleteConstraint(R, Hessian_factor_inv_, A, u, p, iq, ip);

              for (i = 0; i < m; i++)
                iai(i) = i;
              for (i = 0; i < iq; i++)
              {
                A(i) = A_old(i);
                iai(A(i)) = -1;
                u(i) = u_old(i);
              }
              x = x_old;
              goto l2; /* go to step 2 */
            }
            else
              iai(ip) = -1;
            goto l1;
          }

          /* a patial step has taken */
          /* drop constraint l */
          iai(l) = l;
          deleteConstraint(R, Hessian_factor_inv_, A, u, p, iq, l);

          s(ip) = -(CI.row(ip).dot(x) + ci0(ip));

          goto l2a;
      }
  };

}
