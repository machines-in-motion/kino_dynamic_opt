import numpy as np
import matplotlib.pyplot as plt

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import zero

import scipy.linalg

from scipy.interpolate import CubicSpline

from momentumopt.kinoptpy.qp import QpSolver


class SecondOrderInverseKinematics(object):
    def __init__(self, model, endeff_frame_names):

        def getFrameId(name):
            idx = self.robot.model.getFrameId(name)
            if idx == len(self.robot.model.frames):
                raise Exception('Unknown frame name: {}'.format(name))
            return idx

        self.robot = RobotWrapper(model)
        self.robot_mass = sum([i.mass for i in self.robot.model.inertias[1:]])
        self.base_id = getFrameId('base_link')
        self.endeff_frame_names = endeff_frame_names
        self.endeff_ids = [getFrameId(name) for name in endeff_frame_names]
        self.is_init_time = True

        self.ne = len(self.endeff_ids)
        self.nv = self.robot.model.nv

        # Tracking weights
        self.w_endeff_tracking = 10**5
        self.w_endeff_contact = 10**5
        self.w_lin_mom_tracking = 100.0
        self.w_ang_mom_tracking = 10.0
        self.w_joint_regularization = 0.1

        # P and D gains for tracking the position
        self.p_endeff_tracking = 10000.
        self.d_endeff_tracking = 200

        self.p_com_tracking = [10., 10., 10.]
        self.p_orient_tracking = 10.
        self.d_orient_tracking = 1.
        self.p_mom_tracking = 10. * np.array([1., 1., 1., .01, .01, .01])

        self.p_joint_regularization = 1.
        self.d_joint_regularization = .5

        self.desired_acceleration = np.zeros(((self.ne + 2) * 3 + (self.nv - 6), ))


        # Allocate space for the jacobian and desired velocities.
        # Using two entires for the linear and angular velocity of the base.
        # (self.nv - 6) is the number of joints for posture regularization
        self.J = np.zeros(((self.ne + 2) * 3 + (self.nv - 6), self.nv))
        self.drift_terms = np.zeros_like(self.desired_acceleration) #i.e. dJ * dq
        self.measured_velocities = np.zeros((self.J.shape[0], )) # i.e. J * dq

        self.use_hierarchy = False
        self.qp_solver = QpSolver()

    def framesPos(self, frames):
        """
            puts the translations of the list of frames in a len(frames) x 3 array
        """
        return np.vstack([self.robot.data.oMf[idx].translation for idx in frames]).reshape([len(frames),3])


    def update_des_acc(self, q, dq, com_ref, orien_ref, mom_ref, dmom_ref,
                        endeff_pos_ref, endeff_vel_ref, endeff_acc_ref,
                        joint_regularization_ref):

        measured_op_space_velocities = self.J @ dq

        # get the ref momentum acc
        self.desired_acceleration[:6] = dmom_ref + np.diag(self.p_mom_tracking) @ (mom_ref - self.robot.data.hg)

        # com part
        self.desired_acceleration[:3] += np.diag(self.p_com_tracking) @ (com_ref - self.robot.com(q))

        # orientation part
        base_orien = self.robot.data.oMf[self.base_id].rotation
        orient_error = pin.log3(base_orien.T @ orien_ref.matrix()) # rotated in world
        self.desired_acceleration[3:6] += (self.p_orient_tracking * orient_error -
            self.d_orient_tracking * dq[3:6])

        # desired motion of the feet
        for i, idx in enumerate(self.endeff_ids):
            self.desired_acceleration[6 + 3*i: 6 + 3*(i + 1)] = self.p_endeff_tracking * (endeff_pos_ref[i] - self.robot.data.oMf[idx].translation)
            self.desired_acceleration[6 + 3*i: 6 + 3*(i + 1)] += self.d_endeff_tracking*(endeff_vel_ref[i] - measured_op_space_velocities[6 + 3*i: 6 + 3*(i + 1)])
            self.desired_acceleration[6 + 3*i: 6 + 3*(i + 1)] += endeff_acc_ref[i]

        if joint_regularization_ref is None:
            self.desired_acceleration[(self.ne + 2) * 3:] = zero(self.nv - 6)
        else:
            # we add some damping
            self.desired_acceleration[(self.ne + 2) * 3:] = self.p_joint_regularization * (joint_regularization_ref - q[7:])
            self.desired_acceleration[(self.ne + 2) * 3:] += - self.d_joint_regularization * dq[6:]

    def fill_weights(self, endeff_contact):
        w = [self.w_lin_mom_tracking * np.ones(3), self.w_ang_mom_tracking * np.ones(3)]
        for eff in endeff_contact:
            if eff == 1.: # If in contact
                w.append(self.w_endeff_contact * np.ones(3))
            else:
                w.append(self.w_endeff_tracking * np.ones(3))
        w.append(self.w_joint_regularization * np.ones(self.nv - 6))
        self.w = np.diag(np.hstack(w))

    def update_kinematics(self, q, dq):
        # Update the pinocchio model.
        self.robot.forwardKinematics(q, dq)
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        pin.computeJointJacobiansTimeVariation(self.robot.model, self.robot.data, q, dq)
        pin.computeCentroidalMapTimeVariation(self.robot.model, self.robot.data, q, dq)

        # update the op space Jacobian
        # the momentum Jacobian
        self.J[:6, :] = self.robot.data.Ag

        # the feet Jacobians
        for i, idx in enumerate(self.endeff_ids):
            self.J[6 + 3 * i: 6 + 3 * (i + 1), :] = self.robot.getFrameJacobian(idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]

        # this is joint regularization part
        self.J[(self.ne + 2) * 3:,6:] = np.identity(self.nv - 6)

        # update the dJdt dq component aka the drift
        # the momentum drift
        self.drift_terms[:6, ] = self.robot.data.dAg @ dq

        # the feet drift
        for i, idx in enumerate(self.endeff_ids):
            self.drift_terms[6 + 3 * i: 6 + 3 * (i + 1),] = pin.getFrameJacobianTimeVariation(self.robot.model, self.robot.data, idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] @ dq

        # note that the drift of the joints (as a task) is 0


    def step(self, q, dq, com_ref, orien_ref, mom_ref, dmom_ref,
             endeff_pos_ref, endeff_vel_ref, endeff_acc_ref,
             endeff_contact, joint_regularization_ref):
        '''
        Arguments:
            q: Current robot state
            dq: Current robot velocity
            com_ref: Reference com position in global coordinates
            lmom_ref: Reference linear momentum in global coordinates
            amom_ref: Reference angular momentum in global coordinates
            endeff_pos_ref: [N_endeff x 3] Reference endeffectors position in global coordinates
            endeff_vel_ref: [N_endeff x 3] Reference endeffectors velocity in global coordinates
        '''
        self.update_kinematics(q, dq)

        self.update_des_acc(q, dq, com_ref, orien_ref, mom_ref, dmom_ref,
                            endeff_pos_ref, endeff_vel_ref, endeff_acc_ref,
                            joint_regularization_ref)
        self.fill_weights(endeff_contact)

        if self.use_hierarchy:
            J_feet = self.J[6:6+12, :]
            J_feet_pinv = scipy.linalg.pinv(J_feet, cond=0.00001)
            ddq_feet = J_feet_pinv @ (self.desired_acceleration[6:6+12] - self.drift_terms[6:6+12])
            N_feet = np.eye(self.nv) - J_feet_pinv @ J_feet
            J_rest = self.J[:6,:]
            J_rest_pinv = scipy.linalg.pinv(J_rest @ N_feet, cond=0.00001)
            rest_acc = self.desired_acceleration[:6]
            rest_acc = rest_acc - self.drift_terms[:6]
            rest_acc = rest_acc - J_rest @ ddq_feet
            return ddq_feet + J_rest_pinv @ rest_acc

        else:
            hessian = self.J.T @ self.w @ self.J
            hessian += 1e-6 * np.identity(len(hessian))
            gradient = - self.J.T.dot(self.w).dot(self.desired_acceleration - self.drift_terms).reshape(-1)
            return self.qp_solver.quadprog_solve_qp(hessian, gradient)

    def solve(self, dt, q_init, dq_init, com_ref, lmom_ref, amom_ref,
              endeff_pos_ref, endeff_vel_ref, endeff_contact,
              joint_pos_ref, base_ori_ref):

        num_time_steps = com_ref.shape[0]

        com_kin = np.zeros_like(com_ref)
        lmom_kin = np.zeros_like(lmom_ref)
        amom_kin = np.zeros_like(amom_ref)
        endeff_pos_kin = np.zeros_like(endeff_pos_ref)
        endeff_vel_kin = np.zeros_like(endeff_vel_ref)
        q_kin = np.zeros([num_time_steps,q_init.shape[0]])
        dq_kin = np.zeros([num_time_steps,dq_init.shape[0]])
        ddq_kin = np.zeros_like(dq_kin)

        inner_steps = int(dt/0.001)
        inner_dt = 0.001
        time = np.linspace(0., (num_time_steps-1)*dt, num_time_steps)
        splined_com_ref = CubicSpline(time, com_ref)
        splined_lmom_ref = CubicSpline(time, lmom_ref)
        splined_amom_ref = CubicSpline(time, amom_ref)
        splined_endeff_pos_ref = CubicSpline(time, endeff_pos_ref)
        splined_endeff_vel_ref = CubicSpline(time, endeff_vel_ref)
        splined_joint_pos_ref = CubicSpline(time, joint_pos_ref)
        splined_base_ori_ref = CubicSpline(time, base_ori_ref)


        # store the first one
        q = q_init.copy()
        dq = dq_init.copy()
        self.update_kinematics(q, dq)

        q_kin[0] = q
        dq_kin[0] = dq
        com_kin[0] = self.robot.com(q).T
        hg = self.robot.centroidalMomentum(q, dq)
        lmom_kin[0] = hg.linear.T
        amom_kin[0] = hg.angular.T
        endeff_pos_kin[0] = self.framesPos(self.endeff_ids)
        endeff_vel_kin[0] = (self.J[6:(self.ne + 2) * 3].dot(dq).T).reshape([self.ne,3])

        dmom_ref = np.zeros([6,])
        endeff_acc_ref = np.zeros([self.ne,3])
        t = 0.
        for it in range(1,num_time_steps):
            for inner in range(inner_steps):
                dmom_ref = np.hstack((splined_lmom_ref(t, nu=1),
                                   splined_amom_ref(t, nu=1)))
                endeff_acc_ref = splined_endeff_vel_ref(t, nu=1)
                orien_ref = pin.Quaternion(pin.rpy.rpyToMatrix(splined_base_ori_ref(t)))
                ddq = self.step(
                        q, dq, splined_com_ref(t), orien_ref,
                        np.hstack((splined_lmom_ref(t), splined_amom_ref(t))), dmom_ref,
                        splined_endeff_pos_ref(t), splined_endeff_vel_ref(t), endeff_acc_ref,
                        endeff_contact[it], splined_joint_pos_ref(t))

                # Integrate to the next state.
                dq += ddq * inner_dt
                q = pin.integrate(self.robot.model, q, dq * inner_dt)
                t += inner_dt

                self.update_kinematics(q, dq)
            q_kin[it] = q
            dq_kin[it] = dq
            ddq_kin[it] = ddq
            com_kin[it] = self.robot.com(q).T
            hg = self.robot.centroidalMomentum(q, dq)
            lmom_kin[it] = hg.linear.T
            amom_kin[it] = hg.angular.T
            endeff_pos_kin[it] = self.framesPos(self.endeff_ids)
            endeff_vel_kin[it] = (self.J[6:(self.ne + 2) * 3].dot(dq).T).reshape([self.ne,3])

        return q_kin, dq_kin, com_kin, lmom_kin, amom_kin, endeff_pos_kin, endeff_vel_kin
