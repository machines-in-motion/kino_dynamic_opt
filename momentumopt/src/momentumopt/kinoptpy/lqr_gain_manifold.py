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
        # allocate memory for derivative computations
        self.fx = np.zeros((self.N,self.nx, self.nx))
        self.fu = np.zeros((self.N, self.nx, self.m))
        self.cx = np.zeros((self.N,self.nx))
        self.cu = np.zeros((self.N,self.m))
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
        self.x0 = np.zeros((self.N, self.n))
        self.x0[:, :3] = self.com_pos.copy()
        self.x0[:,3:6] = self.com_vel.copy()
        self.x0[:,6:10] = self.com_ori.copy()
        self.x0[:,10:13] = self.com_ang_vel.copy()
        self.u0 = np.zeros((self.N-1, self.m))
        self.u0[:, :3] = self.cent_force[:-1].copy()
        self.u0[:, 3:] = self.cent_moments[:-1].copy()

        self.x0 = self.compute_trajectory_from_controls(self.x0, self.u0)


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
        dq = self.exp_quaternion(.5*self.dt*w)
        return self.quaternion_product(dq,q)

    def quaternion_difference(self, q1, q2):
        """computes the tangent vector from q1 to q2 at Identity """
        # first compute dq s.t.  q2 = q1*dq
        q1conjugate = np.array([-q1[0],-q1[1],-q1[2],q1[3]])
        dq = self.quaternion_product(q1conjugate, q2)
        # increment is log of dq 
        return self.log_quaternion(dq)

    def integrate_position(self, x, u, q):
        # in the commented part below I check if velocity provided
        # is in body or global frame
        return x[:3] + self.dt * u # self.quaternion_to_rotation(q).T.dot(u)

    def integrate_veocity(self, v, f):
        return v + (self.dt/self.mass) * (f - self.weight) 

    def integrate_angular_velocity(self, w, q, tau):
        R = self.quaternion_to_rotation(q)
        factor = linalg.cho_factor(R.dot(self.inertia_com_frame).dot(R.T))
        wnext = w + self.dt * linalg.cho_solve(factor, tau-np.cross(w,np.dot(R.dot(self.inertia_com_frame).dot(R.T),w)))
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
        qnext = self.integrate_quaternion(x[6:10], x[10:13])
        R = self.quaternion_to_rotation(x[6:10])
        factor = linalg.cho_factor(R.dot(self.inertia_com_frame).dot(R.T))
        wnext = x[10:13] + self.dt * linalg.cho_solve(factor, u[3:]-np.cross(x[10:13],np.dot(R.dot(self.inertia_com_frame).dot(R.T),x[10:13])))
        return np.hstack([cnext, vnext, qnext, wnext])

    def increment_x(self, x, dx):
        """ perturbs x with dx """
        #TODO: compute state increments 
        return x 

    def diff_x(self, x2, x1):
        """ return the difference between x2 and x1 as x2 (-) x1 on the manifold """
        #TODO: compute state difference 
        return x2 - x1 

    def dynamics_derivatives(self, t, x, u):
        """ computes df/dx and df/du """
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
            du[i] = 0.
        fu /= self.eps 
        return fx, fu  




    def tracking_cost(self, t, x, u):
        """ a tracking cost for state and controls """
        pass

    def cost_derivatives(self, t, x, u):
        pass

    def compute_gains(self):
        """ includes """
        pass
