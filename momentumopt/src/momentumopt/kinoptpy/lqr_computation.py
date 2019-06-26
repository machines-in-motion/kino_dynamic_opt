import numpy as np
import lqr_gain_manifold
import scipy.linalg as linalg

def compute_terminal_Q():
    lqr_solver = lqr_gain_manifold.CentroidalLqr("../../../../momentumopt/demos")
    N = lqr_solver.N 
    lqr_solver.value[N] = lqr_solver.cost(N, lqr_solver.x0[N], np.zeros(lqr_solver.m))
    cx, cu, cxx, cuu, cxu  = lqr_solver.cost_derivatives(N, lqr_solver.x0[N], np.zeros(lqr_solver.m))
    lqr_solver.value_dx[N,:] = cx.copy()
    lqr_solver.value_dxdx[N,:,:] = cxx.copy()

    Q0 = lqr_solver.cost(N-1,lqr_solver.x0[N-1],lqr_solver.u0[N-1])
    print 'Q0 at N-1 = ', Q0
    fx, fu = lqr_solver.dynamics_derivatives(N-1,lqr_solver.x0[N-1],lqr_solver.u0[N-1])
    cx, cu, cxx, cuu, cxu = lqr_solver.cost_derivatives(N-1,lqr_solver.x0[N-1],lqr_solver.u0[N-1])

    print 'fx \n===================\n', fx, '\n========================'
    print 'fu \n===================\n', fu, '\n========================'

    Qdx = cx + lqr_solver.value_dx[N].dot(fx)
    print 'Qdx \n================\n', Qdx
    Qdu = cu + lqr_solver.value_dx[N].dot(fu)
    print 'Qdu \n================\n', Qdu
    Qdxdx = cxx + fx.T.dot(lqr_solver.value_dxdx[N]).dot(fx)
    print 'Qdxdx \n=====================\n', Qdxdx

    print 'cuu \n===================\n', cuu, '\n========================'
    print 'Vxx_next rank \n===================\n',\
        np.linalg.matrix_rank(lqr_solver.value_dxdx[N]), '\n========================'
    print 'fu.T.Vxx_next.fu rank \n===================\n',\
    np.linalg.matrix_rank(fu.T.dot(lqr_solver.value_dxdx[N]).dot(fu)), '\n========================'




    Qdudu = cuu + fu.T.dot(lqr_solver.value_dxdx[N]).dot(fu)
    print 'Qdudu rank \n===================\n',\
        np.linalg.matrix_rank(Qdudu), '\n========================'


    print 'Qdudu \n=====================\n', Qdudu

    print '\n=========================\nPERFORMING CHOLESKY FACTORIZATION'
    try:
        lQuu = linalg.cho_factor(Qdudu)
    except:
        invQuu = _smooth_inv(Qdudu)
        print 'smooth inverse given by \n============\n', invQuu

def _smooth_inv(m):
    """ adds positive value to eigen values before inverting """
    w, v = np.linalg.eigh(m)
    w_inv = w / (w ** 2 + 1e-2)
    return v.dot(np.diag(w_inv)).dot(v.transpose())


    
    
    






if __name__ == '__main__':
    compute_terminal_Q()