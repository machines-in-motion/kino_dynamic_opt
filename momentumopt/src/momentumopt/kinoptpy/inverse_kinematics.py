import numpy as np
from momentumopt.kinoptpy.qp import QpSolver
from pinocchio import RobotWrapper
import pinocchio as se3
from pinocchio.utils import zero

class PointContactInverseKinematics(object):
    def __init__(self, model, endeff_frame_names):
        def getFrameId(name):
            idx = model.getFrameId(name)
            if idx == len(model.frames):
                raise Exception('Unknown frame name: {}'.format(name))
            return idx

        self.robot = RobotWrapper(model)
        self.model = model
        self.data = self.robot.data
        self.mass = sum([i.mass for i in self.robot.model.inertias[1:]])
        self.base_id = self.robot.model.getFrameId('base_link')
        self.endeff_frame_names = endeff_frame_names
        self.endeff_ids = [getFrameId(name) for name in endeff_frame_names]
        self.is_init_time=1

        self.ne = len(self.endeff_ids)
        self.nv = self.model.nv

        # Tracking weights
        self.w_endeff_tracking = 1.
        self.w_endeff_contact = 1.
        self.w_lin_mom_tracking = 1.0
        self.w_ang_mom_tracking = 1.0
        self.w_joint_regularization = 0.01

        # P gains for tracking the position
        self.p_endeff_tracking = 1.
        self.p_com_tracking = 1.

        self.last_q = None
        self.last_dq = None

        # Allocate space for the jacobian and desired velocities.
        # Using two entires for the linear and angular velocity of the base.
        # (self.nv - 6) is the number of jointss for posture regularization
        self.J = np.zeros(((self.ne + 2) * 3 + (self.nv - 6), self.nv))
        self.vel_des = np.zeros(((self.ne + 2) * 3 + (self.nv - 6), 1))

        self.qp_solver = QpSolver()

    def rotate_J(self, jac, index):
        world_R_joint = se3.SE3(self.data.oMf[index].rotation, zero(3))
        return world_R_joint.action.dot(jac)

    def get_world_oriented_frame_jacobian(self, q, index):
        return self.rotate_J(
            se3.getFrameJacobian(self.model, self.data, index, se3.ReferenceFrame.LOCAL),
            index)

    def fill_jacobians(self, q):
        # REVIEW(jviereck): The Ludo invkin sets the angular momentum part to the identity.
        self.J[:6, :] = self.robot.data.Ag
#         self.J[:3, :] = robot.data.Jcom * invkin.mass
        for i, idx in enumerate(self.endeff_ids):
            self.J[6 + 3 * i: 6 + 3 * (i + 1), :] = self.get_world_oriented_frame_jacobian(q, idx)[:3]
        # this is joint regularization part
        self.J[(self.ne + 2) * 3:,6:] = np.identity(self.nv - 6)
        #print "jac:\n",self.J,"\n\n"


    def fill_vel_des(self, q, dq, com_ref, lmom_ref, amom_ref, endeff_pos_ref, endeff_vel_ref):
        self.vel_des[:3] = (lmom_ref + self.p_com_tracking * (com_ref - self.robot.com(q).T)).T
        self.vel_des[3:6] = amom_ref.reshape(3, 1)

        for i, idx in enumerate(self.endeff_ids):
            if self.is_init_time:
                self.vel_des[6 + 3*i: 6 + 3*(i + 1)] = endeff_vel_ref[i].reshape((3, 1)) + \
                    1. * (endeff_pos_ref[i] - self.robot.data.oMf[idx].translation.T).T
            else:
                self.vel_des[6 + 3*i: 6 + 3*(i + 1)] = endeff_vel_ref[i].reshape((3, 1)) + \
                    self.p_endeff_tracking * (
                        endeff_pos_ref[i] - self.robot.data.oMf[idx].translation.T).T
        self.vel_des[(self.ne + 2) * 3:] = zero(self.nv - 6)
        #print "vel:\n",self.vel_des,"\n\n"

    def fill_weights(self, endeff_contact):
        w = [self.w_lin_mom_tracking * np.ones(3), self.w_ang_mom_tracking * np.ones(3)]
        for eff in endeff_contact:
            if eff == 1.: # If in contact
                w.append(self.w_endeff_contact * np.ones(3))
            else:
                w.append(self.w_endeff_tracking * np.ones(3))
        w.append(self.w_joint_regularization * np.ones(self.nv - 6))
        self.w = np.diag(np.hstack(w))
        #print "w:\n",self.w,"\n\n"


    def forward_robot(self, q, dq):
        # Update the pinocchio model.
        self.robot.forwardKinematics(q, dq)
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

        self.last_q = q.copy()
        self.last_dq = dq.copy()

    def compute(self, q, dq, com_ref, lmom_ref, amom_ref, endeff_pos_ref, endeff_vel_ref, endeff_contact):
        """
        Arguments:
            q: Current robot state
            dq: Current robot velocity
            com_ref: Reference com position in global coordinates
            lmom_ref: Reference linear momentum in global coordinates
            amom_ref: Reference angular momentum in global coordinates
            endeff_pos_ref: [N_endeff x 3] Reference endeffectors position in global coordinates
            endeff_vel_ref: [N_endeff x 3] Reference endeffectors velocity in global coordinates
        """
        if not np.all(np.equal(self.last_q, q)) or np.all(np.equal(self.last_dq, dq)):
            self.forward_robot(q, dq)

        self.fill_jacobians(q)
        self.fill_vel_des(q, dq, com_ref, lmom_ref, amom_ref, endeff_pos_ref, endeff_vel_ref)
        self.fill_weights(endeff_contact)

        self.forwarded_robot = False
        # print np.shape(self.w)
        # print np.shape(self.J),"\n"
        hessian = self.J.T.dot(self.w).dot(self.J)
        # print hessian, "\n"
        hessian += 1e-6 * np.identity(len(hessian))
        gradient = -self.J.T.dot(self.w).dot(self.vel_des).reshape(-1)
        w,v=np.linalg.eig(hessian)
        # np.set_printoptions(precision=6)
        # print w,"\n"
        return np.matrix(self.qp_solver.quadprog_solve_qp(hessian, gradient)).T
