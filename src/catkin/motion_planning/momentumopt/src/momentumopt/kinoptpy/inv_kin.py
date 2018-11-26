import numpy as np
import pinocchio as se3
# from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from numpy.linalg import pinv
from src.momentumopt.kinoptpy.qp import QpSolver

class InverseKinematics:
    def __init__(self, dt, nq):
        self.dt = dt
        self.desired_vels = []
        self.jacobians = []
        self.K_p = np.zeros((0))
        self.weights = []

        self.centroidal_momentum = None

        # create QP-solver object
        self.qp_solver = QpSolver()

        self.lambda_ = 0.0001 * np.ones((nq - 1))

    def get_dt(self):
        return self.dt

    def add_task(self, desired_velocity, jacobian, p_gain, weight=1.0):
        self.desired_vels.append(desired_velocity)
        self.jacobians.append(jacobian)
        self.weights.append(weight)
        K_p_ = np.ones((self.K_p.shape[0] + 1))
        K_p_[-1] = p_gain
        K_p_[:-1] = self.K_p
        self.K_p = K_p_

    def add_tasks(self, desired_velocities, jacobians, centroidal_momentum=None, desired_momentums=None, gains=1.0, weights=None):
        self.centroidal_momentum = centroidal_momentum
        self.desired_momentums = desired_momentums

        if not len(desired_velocities) == len(jacobians): 
            raise ValueError("%d != %d. The number of desired velocities and jacobians have to be equal." %(len(desired_velocities), len(jacobians)))
        if not isinstance(gains, float):
            if len(jacobians) == len(gains):
                raise ValueError("%d != %d. The number of tasks and gains have to be equal or specify one gain for all tasks." %(len(jacobians), len(gains)))

        for i in range(len(desired_velocities)):
            if isinstance(gains, float):
                gain = gains
            else:
                gain = gains[i]

            if not weights is None:
                self.add_task(desired_velocities[i], jacobians[i], gain, weights[i])
            else:
                self.add_task(desired_velocities[i], jacobians[i], gain)

    def delete_tasks(self):
        self.desired_vels = []
        self.jacobians = []
        self.K_p = np.zeros((0))
        self.weights = []

    def set_regularizer(self, lambda_):
        self.lambda_ = lambda_

    # Multi task inverse kinematics using pseudo inverse to minimize sum(||J_i * q_dot - x_i_dot ||^2)
    def multi_task_IK_pseudo_inverse(self, q):
        vq = pinv(self.jacobians[0](q)) * self.desired_vels[0]()

        if len(self.desired_vels) > 1:
            stacked_jacobians = self.jacobians[0](q)
            for i in range(len(self.desired_vels) - 1):
                if i > 0:
                    stacked_jacobians = np.concatenate((stacked_jacobians, self.jacobians[i](q)))
                null_space_proj = eye(stacked_jacobians.shape[1]) - pinv(stacked_jacobians) * stacked_jacobians
                vq += null_space_proj * pinv(self.jacobians[i + 1](q) * null_space_proj) * (self.desired_vels[i + 1]() - self.jacobians[i + 1](q) * vq)

        return vq

    # Multi task inverse kinematics using QP to minimize sum(||J_i * q_dot - x_i_dot ||^2)
    def multi_task_IK(self, G=None, h=None, soft_constrained=False):
        # Transform the multi task inverse kinematics into QP 
        # min 0.5 * q_dot^T * P * q_dot + r^T * q_dot
        # with:
        # P = sum(2.0 * J_i^T * J_i)
        # r = sum(- 2.0 * J_i^T * x_i_dot)

        for i in range(len(self.jacobians)):
            J_ = self.jacobians[i]()
            if i == 0:
                P = self.weights[i] * 2.0 * np.dot(np.transpose(J_), J_)
                r = self.weights[i] * (- 2.0) * np.squeeze(np.array(np.dot(np.transpose(J_), self.K_p[i] * self.desired_vels[i](self.dt)))) 
            else:
                P += self.weights[i] * 2.0 * np.dot(np.transpose(J_), J_)
                r += self.weights[i] * (- 2.0) * np.squeeze(np.array(np.dot(np.transpose(J_), self.K_p[i] * self.desired_vels[i](self.dt)))) 

        # if not self.centroidal_momentum is None: 
        #     # weight_momentum = 0.0001  # 1.0 * self.weights[0]
        #     weight_momentum = 0.01 * np.ones((6))
        #     weight_momentum[3:] = 0.0
        #     K_p_momentum = 1.0  # self.K_p[0]
        #     H_ = self.centroidal_momentum()
        #     P += 2.0 * np.dot(np.transpose(H_), np.dot(np.diag(weight_momentum), H_))
        #     r += (- 2.0) * np.squeeze(np.array(np.dot(np.transpose(H_), np.dot(np.diag(weight_momentum), K_p_momentum * self.desired_momentums)))) 
        #     # P = weight_momentum * 2.0 * np.dot(np.transpose(H_), H_)
        #     # r = weight_momentum * (-2.0) * np.squeeze(np.array(np.dot(np.transpose(H_), K_p_momentum * self.desired_momentums))) 

        # Add regularization term, since P is only guranteed to be positive semi-definite
        # P = P + lambda * I
        regularizer = np.diag(self.lambda_)    
        P += regularizer

        P_orig_size = P.shape[0]

        if soft_constrained and not G is None and not h is None:
            P, r, G, h = self.create_soft_constraints(P, r, G, h)

        # Feed P and r to QP solver
        q_sol = self.qp_solver.quadprog_solve_qp(P, r, G=G, h=h)
        q_sol = q_sol[:P_orig_size]

        return q_sol

    def create_soft_constraints(self, P, r, G, h, v=0.001*10, u=0.001*100):
        G_soft_constrained = np.zeros((G.shape[0] + G.shape[0], G.shape[1] + G.shape[0]))
        G_soft_constrained[:G.shape[0], :G.shape[1]] = G
        G_soft_constrained[:G.shape[0], G.shape[1] :] = - eye((G.shape[0]))
        G_soft_constrained[G.shape[0]:, G.shape[1] :] = - eye((G.shape[0]))

        stacked_zeros = np.zeros((h.shape[0], 1))
        h_soft_constrained = np.vstack((np.reshape(h, (h.shape[0], 1)), stacked_zeros))
        h_soft_constrained = np.reshape(h_soft_constrained, ((h_soft_constrained.shape[0])))

        P_soft_constrained = np.zeros((G_soft_constrained.shape[1], G_soft_constrained.shape[1]))
        P_soft_constrained[:P.shape[0], :P.shape[0]] = P

        P_soft_constrained[P.shape[0]:, P.shape[0]:] = v * np.eye((G_soft_constrained.shape[1] - P.shape[0]))

        r_soft_constrained = np.zeros(G_soft_constrained.shape[1])
        r_soft_constrained[:P.shape[0]] = r
        r_soft_constrained[P.shape[0]:] = u

        # update variables
        return P_soft_constrained, r_soft_constrained, G_soft_constrained, h_soft_constrained
