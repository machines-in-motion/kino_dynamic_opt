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
        self.N = self.com_pos.shape[0]
        self.n = 13
        self.m = 6
        self.nx = self.n - 1
        # allocate memory for computed gains 
        self.K = np.zeros((self.N, self.m, self.nx))
        # check if quaternions from planner are unitary
        for t in range(self.N):
            q = se3.Quaternion(self.com_ori[t, 3],
                               self.com_ori[t, 0],
                               self.com_ori[t, 1],
                               self.com_ori[t, 2])
            assert (se3.Quaternion.norm(q)-1.)**2 <= self.eps, \
            'Quaternions are not unitary'
        # initial trajectory
        self.xp = np.zeros((self.N, self.n))
        self.xp[:, :3] = self.com_pos.copy()
        self.xp[:,3:6] = self.com_vel.copy()
        self.xp[:,6:10] = self.com_ori.copy()
        self.xp[:,10:13] = self.com_ang_vel.copy()
        self.u0 = np.zeros((self.N-1, self.m))
        self.u0[:, :3] = self.cent_force[:-1].copy()
        self.u0[:, 3:] = self.cent_moments[:-1].copy()
        # the line below integrates the full trajectory from the given controls
        # removing it sets initial trajectory to the one computed from kinodynamic planner   
        self.x0 = self.compute_trajectory_from_controls(self.xp.copy(), self.u0)
        # defining cost weights 
        self.Q = [np.eye(self.nx)]*self.N
        self.R = [1.e-3*np.eye(self.m)]*(self.N-1) 


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
        for t in range(self.N-1):
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

    def tracking_cost(self, t, x, u):
        """ a tracking cost for state and controls 
            here we assume x0 to be the reference trajectory """
        dx = self.diff_x(x, self.x0[t])
        if t == self.N-1:
            return  dx.T.dot(self.Q[t]).dot(dx)
        else:
            return dx.T.dot(self.Q[t]).dot(dx) + u.T.dot(self.R[t].dot(u))
        # else:
        #     raise BaseException, 'time index is greater than the horizon'

    def cost_derivatives(self, t, x, u):
        """ compute cx, cxx, cu, cuu, cxu using finite differences """
        c0 = self.tracking_cost(t, x, u)
        cx = np.zeros(self.nx)
        cu = np.zeros(self.m)
        dx = np.zeros(self.nx)
        # compute gradients 
        for i in range(self.nx): 
            dx[i] = self.eps 
            cp = self.tracking_cost(t, self.increment_x(x,dx), u)
            cx[i] = cp - c0 
            dx[i] = 0. 
        cx /= self.eps 
        du = np.zeros(self.m)
        for i in range(self.m):
            du[i] = self.eps
            cp = self.tracking_cost(t, x, u+du)
            cu[i] = cp-c0
            du[i] = 0. 
        cu /= self.eps 
        # compute hessians 

        return cx, cu




    def compute_gains(self):
        """ performs a backward pass to compute the lqr gain for the 
        reference trajectories """

        # initialize value function at terminal state 
        # do backward recursion to compute the trajectory gains 
        pass
