'''
@file qp.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

from numpy import *
# from cvxopt import matrix, spmatrix
# from cvxopt.solvers import options, qp
# from cvxpy import Constant, Minimize, Problem, Variable, quad_form
from quadprog import solve_qp


class QpSolver():
    def quadprog_solve_qp(self, P, q, G=None, h=None, A=None, b=None, initvals=None):
        '''
        Solve a Quadratic Program defined as:

            minimize
                (1/2) * x.T * P * x + q.T * x

            subject to
                G * x <= h
                A * x == b

        using quadprog <https://pypi.python.org/pypi/quadprog/>.

        Parameters
        ----------
        P : numpy.array
            Symmetric quadratic-cost matrix.
        q : numpy.array
            Quadratic-cost vector.
        G : numpy.array
            Linear inequality constraint matrix.
        h : numpy.array
            Linear inequality constraint vector.
        A : numpy.array, optional
            Linear equality constraint matrix.
        b : numpy.array, optional
            Linear equality constraint vector.
        initvals : numpy.array, optional
            Warm-start guess vector (not used).

        Returns
        -------
        x : numpy.array
            Solution to the QP, if found, otherwise ``None``.

        Note
        ----
        The quadprog solver only considers the lower entries of `P`, therefore it
        will use a wrong cost function if a non-symmetric matrix is provided.
        '''
        if initvals is not None:
            print("quadprog: note that warm-start values ignored by wrapper")
        qp_G = P
        qp_a = -q
        if A is not None and G is None:
            meq = A.shape[0]
            return solve_qp(qp_G, qp_a, -A.T, -b, meq)[0]
        elif G is not None:
            if A is not None:
                qp_C = -vstack([A, G]).T
                qp_b = -hstack([b, h])
                meq = A.shape[0]
                # print("EQUALITY AND INEQUALITY CONSTRAINTS")
            else:   # no equality constraint
                qp_C = -G.T
                qp_b = -h
                meq = 0
                # print("NO EQUALITY CONSTRAINT")
            return solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]
        else:
            # print("UNCONSTRAINED OPTIMIZATION")
            return solve_qp(qp_G, qp_a)[0]

    def cvxopt_solve_qp(self, P, q, G=None, h=None, A=None, b=None):
        # P = .5 * (P + P.T)  # make sure P is symmetric
        args = [matrix(P), matrix(q)]
        if G is not None:
            args.extend([matrix(G), matrix(h)])
            if A is not None:
                args.extend([matrix(A), matrix(b)])
        sol = qp(*args)
        if 'optimal' not in sol['status']:
            return None
        return array(sol['x']).reshape((P.shape[1],))
