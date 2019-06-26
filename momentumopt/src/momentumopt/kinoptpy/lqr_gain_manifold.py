### computes gains using lqr in the centroidal space for solo (assumes legs are weightless)
### Performs a backward pass to compute gains using a trajectory
### Author: Bilal Hammoud 
### Date:6/5/2019


import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
# from scipy.spatial.transform import Rotation as Rot
import scipy.linalg as linalg
import pinocchio as se3


np.set_printoptions(linewidth=13000)

class CentroidalLqr:

    def __init__(self, dir):

        self.dir = dir
        self.com_pos = np.loadtxt(dir + "/quadruped_com.dat", dtype=float)[:, [1,2,3]]
        self.com_vel = np.loadtxt(dir + "/quadruped_com_vel.dat", dtype=float)[:, [1,2,3]]
        self.com_ori = np.loadtxt(dir + "/quadruped_quaternion.dat", dtype=float)[:, [1,2,3,4]]
        self.com_ang_vel = np.loadtxt(dir + "/quadruped_base_ang_velocities.dat", dtype=float)[:, [1,2,3]]
        #
        self.cent_force = np.loadtxt(dir + "/quadruped_centroidal_forces.dat", dtype=float)[:, [1,2,3]]
        self.cent_moments = np.loadtxt(dir + "/quadruped_centroidal_moments.dat", dtype=float)[:, [1,2,3]]

        #
        self.eps = 1.e-6
        self.dt = 1.e-3
        self.mass = 2.17
        self.base_mass = 1.43315
        self.inertia_com_frame = [[0.00578574, 0.0, 0.0],
                                  [0.0, 0.01938108, 0.0],
                                  [0.0, 0.0, 0.02476124]]
        self.inertia_com_frame_scaled = np.dot(self.mass/self.base_mass,self.inertia_com_frame)
        self.weight = self.mass * np.array([0., 0., 9.81])
        # state control and error dimensions
        self.N = self.com_pos.shape[0]-1 
        self.n = 13
        self.m = 6
        self.nx = self.n - 1
        # check if quaternions from planner are unitary
        for t in range(self.N):
            q = se3.Quaternion(self.com_ori[t, 3],
                               self.com_ori[t, 0],
                               self.com_ori[t, 1],
                               self.com_ori[t, 2])
            assert (se3.Quaternion.norm(q)-1.)**2 <= self.eps, \
            'Quaternions are not unitary'
        # initial trajectory
        self.xp = np.zeros((self.N+1, self.n))
        self.xp[:, :3] = self.com_pos.copy()
        self.xp[:,3:6] = self.com_vel.copy()
        self.xp[:,6:10] = self.com_ori.copy()
        self.xp[:,10:13] = self.com_ang_vel.copy()
        self.u0 = np.zeros((self.N, self.m))
        self.u0[:, :3] = self.cent_force[:-1].copy()
        self.u0[:, 3:] = self.cent_moments[:-1].copy()
        # the line below integrates the full trajectory from the given controls
        # removing it sets initial trajectory to the one computed from kinodynamic planner   
        self.x0 = self.compute_trajectory_from_controls(self.xp.copy(), self.u0)
        # defining cost weights 
        self.Q = [np.eye(self.nx)]*(self.N+1)
        self.R = [1.e-3*np.eye(self.m)]*self.N 

        # allocate value function and gain arrays 
        self.value = np.zeros(self.N+1)
        self.value_dx = np.zeros((self.N+1, self.nx))
        self.value_dxdx = np.zeros((self.N+1, self.nx, self.nx))
        self.Q0 = np.zeros(self.N+1)
        self.Qdx = np.zeros((self.N+1, self.nx))
        self.Qdu = np.zeros((self.N+1, self.m))
        self.Qdxdx = np.zeros((self.N+1, self.nx, self.nx))
        self.Qdudu = np.zeros((self.N+1, self.m, self.m))
        self.Qdxdu = np.zeros((self.N+1, self.nx, self.m))
        self.kfb = np.zeros((self.N, self.m, self.nx))
        self.kff = np.zeros((self.N, self.m))




    def skew(self, v):
        '''converts vector v to skew symmetric matrix'''
        assert v.shape[0] == 3, 'vector dimension is not 3 in skew method'
        return np.array([[0., -v[2], v[1]],
                         [v[2], 0., -v[0]],
                         [-v[1], v[0], 0.]])

    def quaternion_to_rotation(self, q):
        ''' converts quaternion to rotation matrix '''
        return (q[3]**2 - q[:3].dot(q[:3]))*np.eye(3) \
            + 2. * np.outer(q[:3], q[:3]) + 2.*q[3]*self.skew(q[:3])

    def exp_quaternion(self, w):
        ''' converts angular velocity to quaternion '''
        qexp = np.zeros(4)
        th = np.linalg.norm(w)
        if th**2 <= self.eps:
            ''' small norm causes closed form to diverge,
            use taylor expansion to approximate '''
            qexp[:3] =(1-(th**2)/6)*w
            qexp[3] = 1-(th**2)/2
        else:
            u = w/th
            qexp[:3] = np.sin(th)*u
            qexp[3] = np.cos(th)
        return qexp

    def log_quaternion(self, q):
        """ lives on the tangent space of SO(3) """
        v = q[:3]
        w = q[3]
        vnorm = np.linalg.norm(v)
        if vnorm <= self.eps:
            return 2 * v/w * (1 - vnorm**2/(3*w**2))
        else:
            return 2*np.arctan2(vnorm, w) * v / vnorm  

    def quaternion_product(self, q1, q2):
        """ computes quaternion product of q1 x q2 """
        p = np.zeros(4)
        p[:3] = np.cross(q1[:3], q2[:3]) + q2[3]*q1[:3] + q1[3]*q2[:3]
        p[3] = q1[3]*q2[3] - q1[:3].dot(q2[:3])
        return p

    def integrate_quaternion(self, q, w):
        """ updates quaternion with tangent vector w """
        dq = self.exp_quaternion(.5*w)
        return self.quaternion_product(dq,q)

    def quaternion_difference(self, q1, q2):
        """computes the tangent vector from q1 to q2 at Identity
        returns vecotr w  
        s.t. q2 = exp(.5 * w)*q1  
         """
        # first compute dq s.t.  q2 = q1*dq
        q1conjugate = np.array([-q1[0],-q1[1],-q1[2],q1[3]])
        # order of multiplication is very essential here 
        dq = self.quaternion_product(q2, q1conjugate)
        # increment is log of dq  
        return self.log_quaternion(dq)   

    def integrate_veocity(self, v, f):
        return v + (self.dt/self.mass) * (f - self.weight) 

    def integrate_angular_velocity(self, w, q, tau):
        R = self.quaternion_to_rotation(q)
        factor = linalg.cho_factor(R.dot(self.inertia_com_frame).dot(R.T))
        wnext = w + self.dt * linalg.cho_solve(factor, tau-np.cross(w,
                        np.dot(R.dot(self.inertia_com_frame).dot(R.T),w)))
        return wnext

    def compute_trajectory_from_controls(self, x, u):
        ''' xu0 contains initial trajectory at x[0], and controls for the full horizon '''
        for t in range(self.N):
            x[t+1] = self.integrate_step(t, x[t], u[t])
        return x.copy() 


    def integrate_step(self, t, x, u):
        """ state vector x is given by x = [c, v, q, w] where
        c: center of mass cartesian position
        v: center of mass linear velocity
        q: base orientation represented as a quaternion
        w: base angular velocity
         """
        cnext = x[:3] + self.dt * x[3:6]
        vnext = x[3:6] + (self.dt/self.mass) * (u[:3] - self.weight) 
        qnext = self.integrate_quaternion(x[6:10], self.dt * x[10:13])
        R = self.quaternion_to_rotation(x[6:10])
        factor = linalg.cho_factor(R.dot(self.inertia_com_frame).dot(R.T))
        wnext = x[10:13] + self.dt * linalg.cho_solve(factor, u[3:]-np.cross(x[10:13],
                             np.dot(R.dot(self.inertia_com_frame).dot(R.T),x[10:13])))
        return np.hstack([cnext, vnext, qnext, wnext])

    def increment_x(self, x, dx):
        """ perturbs x with dx """
        # com pos
        dc = x[:3] + dx[:3]
        # com vel 
        dv = x[3:6] + dx[3:6]
        # base orientation 
        dq = self.integrate_quaternion(x[6:10], dx[6:9])
        # base angular vel 
        dw = x[10:] + dx[9:]
        return np.hstack([dc, dv, dq, dw])

    def diff_x(self, x1, x2):
        """ return the difference between x2 and x1 as x2 (-) x1 on the manifold 
        """
        dcv = x2[:6] - x1[:6]
        dq = self.quaternion_difference(x1[6:10], x2[6:10])  
        dw = x2[10:] - x1[10:]
        return np.hstack([dcv, dq, dw])

    def dynamics_derivatives(self, t, x, u):
        """ computes df/dx and df/du using finite differentiation """
        fx = np.zeros((self.nx, self.nx))
        fu = np.zeros((self.nx, self.m))
        dx = np.zeros(self.nx)
        f0 = self.integrate_step(t,x,u)
        for i in range(self.nx): 
            dx[i] = self.eps 
            fp = self.integrate_step(t, self.increment_x(x,dx), u)
            fx[:,i] = self.diff_x(fp, f0)
            dx[i] = 0. 
        fx /= self.eps 
        du = np.zeros(self.m)
        for i in range(self.m): 
            du[i] = self.eps 
            fp = self.integrate_step(t, x, u+du)
            fu[:,i] = self.diff_x(fp, f0)
            du[i] = 0.
        fu /= self.eps 
        return fx, fu  

    def cost(self, t, x, u):
        """ a tracking cost for state and controls 
            here we assume x0 to be the reference trajectory """
        dx = self.diff_x(self.x0[t], x)
        if t == self.N:
            return  dx.T.dot(self.Q[t]).dot(dx)
        else:
            return .5 * dx.T.dot(self.Q[t]).dot(dx) + .5 * u.T.dot(self.R[t]).dot(u)
        # else:
        #     raise BaseException, 'time index is greater than the horizon'

    def cost_derivatives(self, t, x, u):
        """ compute cx, cxx, cu, cuu, cxu using finite differences """
        c0 = self.cost(t, x, u)
        cx = np.zeros(self.nx)
        cu = np.zeros(self.m)
        dx = np.zeros(self.nx)
        # compute gradients 
        for i in range(self.nx): 
            dx[i] = self.eps 
            cp = self.cost(t, self.increment_x(x,dx), u)
            cx[i] = cp - c0 
            dx[i] = 0. 
        cx /= self.eps 
        du = np.zeros(self.m)
        for i in range(self.m):
            du[i] = self.eps
            cp = self.cost(t, x, u+du)
            cu[i] = cp-c0
            du[i] = 0. 
        cu /= self.eps 
        # compute hessians 
        cxx = self.cost_dxdx(t, x, u)
        cuu = self.cost_dudu(t, x, u)
        cxu = self.cost_dxdu(t,x,u)

        return cx, cu, cxx, cuu, cxu   

    def cost_dxdx(self, t, x, u):
        x0 = x.copy()
        cxx = np.zeros((self.nx, self.nx))
        dx1 = np.zeros(self.nx)
        dx2 = np.zeros(self.nx)
        for row in range(self.nx):
            dx1[row] += self.eps
            dx2[row] -= self.eps 
            for col in range(self.nx):
                dx1[col] += self.eps
                spp = self.cost(t, self.increment_x(x,dx1), u)
                dx1[col] -= 2*self.eps 
                spm = self.cost(t, self.increment_x(x,dx1), u)
                dx1[col] -= self.eps 
                # 
                dx2[col] += self.eps
                smp = self.cost(t, self.increment_x(x,dx2), u)
                dx2[col] -= 2*self.eps
                smm = self.cost(t, self.increment_x(x,dx2), u)
                dx2[col] += self.eps
                cxx[row,col] = spp-spm-smp+smm
            dx1[row] -= self.eps
            dx2[row] += self.eps
        cxx /= (4*self.eps**2)
        return cxx #  .5 * (cxx + cxx.T) 

    def cost_dudu(self, t, x ,u): 
        cuu = np.zeros((self.m, self.m))
        u1 = u.copy()
        u2 = u.copy() 
        for row in range(self.m):
            u1[row] += self.eps
            u2[row] -= self.eps
            for col in range(self.m):
                u1[col] += self.eps
                spp = self.cost(t, x, u1)
                u1[col] -= 2*self.eps
                spm = self.cost(t, x, u1)
                u1[col] += self.eps
                # 
                u2[col] += self.eps
                smp = self.cost(t, x, u2)
                u2[col] -= 2 * self.eps
                smm = self.cost(t, x, u2)
                u2[col] += self.eps
                cuu[row, col] = spp -spm - smp + smm
            u1[row] -= self.eps
            u2[row] += self.eps
        cuu /= (4*self.eps**2)
        return  cuu #.5 * (cuu + cuu.T)

    def cost_dxdu(self, t, x, u):
        cxu = np.zeros((self.nx, self.m))
        dx1 = np.zeros(self.nx)
        dx2 = np.zeros(self.nx)
        u1 = u.copy()
        u2 = u.copy()
        for col in range(self.m):
            u1[col] += self.eps
            u2[col] -= self.eps
            for row in range(self.nx):
                dx1[row] += self.eps
                spp = self.cost(t, self.increment_x(x,dx1), u1)
                dx1[row] -= 2 * self.eps
                spm = self.cost(t, self.increment_x(x,dx1), u1)
                dx1[row] += self.eps
                dx2[row] += self.eps
                smp = self.cost(t, self.increment_x(x,dx2), u2)
                dx2[row] -= 2 * self.eps
                smm = self.cost(t, self.increment_x(x,dx2), u2)
                dx2[row] += self.eps
                cxu[row, col] = spp - spm - smp + smm
            u1[col] -= self.eps
            u2[col] += self.eps
        cxu /= (4*self.eps**2)
        return cxu 

    def compute_Q(self, V_next, Vx_next, Vxx_next, t, x, u):
        self.Q0[t] = self.cost(t,x,u) + V_next
        fx, fu = self.dynamics_derivatives(t,x,u)
        cx, cu, cxx, cuu, cxu = self.cost_derivatives(t,x,u)
        self.Qdx[t,:] = cx + Vx_next.dot(fx) 
        self.Qdu[t,:] = cu + Vx_next.dot(fu)
        self.Qdxdx[t,:,:] = cxx + fx.T.dot(Vxx_next).dot(fx)
        self.Qdudu[t,:,:] = cuu + fu.T.dot(Vxx_next).dot(fu)
        self.Qdxdu[t,:,:] = cxu + fx.T.dot(Vxx_next).dot(fu)
        
    @staticmethod
    def _smooth_inv(m):
        """ adds positive value to eigen values before inverting """
        w, v = np.linalg.eigh(m)
        w_inv = w / (w ** 2 + 1e-2)
        return v.dot(np.diag(w_inv)).dot(v.transpose())
    
    def compute_value_function(self, t):
        try:
            lQuu = linalg.cho_factor(self.Qdudu[t])
            self.kff[t,:] = linalg.cho_solve(lQuu, self.Qdu[t])
            self.kfb[t,:,:] = linalg.cho_solve(lQuu, self.Qdxdu[t].T)
        except:
            print 'smoothing '
            invQuu = self._smooth_inv(self.Qdudu[t])
            self.kff[t,:] = invQuu.dot(self.Qdu[t])
            self.kfb[t,:,:] =  invQuu.dot(self.Qdxdu[t].T)

        
        self.value[t] = self.Q0[t] + .5 * self.kff[t].T.dot(self.Qdudu[t]).dot(self.kff[t]) \
                       - self.kff[t].T.dot(self.Qdu[t])
        #
        self.value_dx[t,:] = self.Qdx[t] - 2.*self.Qdu[t].dot(self.kfb[t]) \
                        + self.kff[t].dot(self.Qdudu[t]).dot(self.kfb[t])   
        # 
        self.value_dxdx[t, :, :] = self.Qdxdx[t] - self.Qdxdu[t].dot(self.kfb[t])



    def compute_gains(self):
        """ performs a backward pass to compute the lqr gain for the 
        reference trajectories """

        N = self.N
        self.value[N] = self.cost(N, self.x0[N], np.zeros(self.m))
        cx, cu, cxx, cuu, cxu  = self.cost_derivatives(N, self.x0[N], np.zeros(self.m))
        self.value_dx[N,:] = cx.copy()
        self.value_dxdx[N,:,:] = cxx.copy()
        self.compute_Q(self.value[N], self.value_dx[N], self.value_dxdx[N], 
                        N-1, self.x0[N-1],self.u0[N-1])
        self.compute_value_function(N-1)

        # create a loop to compute Q and value functions recursively 
        for t in range(self.N-2,-1, -1):
            print 'performing backward pass at node %s'%t
            self.compute_Q(self.value[t+1], self.value_dx[t+1], self.value_dxdx[t+1],
                            t, self.x0[t], self.u0[t])
            # gains are internally stored in compute_value_fcn 
            self.compute_value_function(t)
                

         
        
