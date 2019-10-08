'''
@file quadruped_wrapper.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import os

import numpy as np
import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero
from robot_properties_solo.config import SoloConfig, Solo12Config

class BasicRobotWrapper(object):

    def __init__(self):
        self.model = None
        self.data = None
        self.q = None

    def set_configuration(self, q):
        self.q = q
        pinocchio.forwardKinematics(self.model, self.data, self.q)

    def set_velocity(self, dq):
        self.dq = dq

    def set_acceleration(self, ddq):
        self.ddq = ddq

    def update_configuration(self, delta_q):
        self.q = pinocchio.integrate(self.model, self.q, delta_q)

        # pinocchio.forwardKinematics(self.model, self.data, self.q)
        # pinocchio.framesKinematics(self.model, self.data)
    def get_difference(self, q_1, q_2):
        return pinocchio.difference(self.model, q_1, q_2)

    def get_world_oriented_frame_jacobian(self, index):
        self.robot.forwardKinematics(self.q, self.dq)
        self.robot.computeJointJacobians(self.q)
        self.robot.framesForwardKinematics(self.q)
        jac = pinocchio.getFrameJacobian(self.model, self.data, index, pinocchio.ReferenceFrame.LOCAL)
        world_R_joint = pinocchio.SE3(self.data.oMf[index].rotation, zero(3))
        return world_R_joint.action.dot(jac)

    def get_jacobian(self, name, dofs=None, internal=True):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Joint %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Joint %s is not available." %name)

        range_ = None
        if dofs == "TRANSLATION":
            range_ = range(3)
        elif dofs == "ROTATION":
            if name == "COM":
                raise ValueError("No rotation for COM available")
            range_ = range(3, 6)
        else:
            range_ = range(6)

        if internal:
            if name == "COM":
                def eval_jac_internal_com():
                    return self.robot.Jcom(self.q)
                return eval_jac_internal_com
            else:
                index = self.model.getFrameId(name)
                def eval_jac_internal():
                    return self.get_world_oriented_frame_jacobian(index)[range_, :]
                    # return pinocchio.frameJacobian(self.model, self.data, self.q, index, pinocchio.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_internal
        else:
            if name == "COM":
                return self.Jcom
            else:
                index = self.model.getFrameId(name)
                def eval_jac_at_q(q):
                    return self.get_world_oriented_frame_jacobian(index)[range_, :]
                    # return pinocchio.frameJacobian(self.model, self.data, q, index, pinocchio.ReferenceFrame.LOCAL)[range_, :]
                return eval_jac_at_q

    def get_centroidal_momentum(self):
        def eval_centroidal_momentum():
            self.robot.centroidalMomentum(self.q, self.dq)
            centroidal_momentum_matrix = self.data.Ag
            return centroidal_momentum_matrix

        return eval_centroidal_momentum

    def get_d_centroidal_momentum(self):
        def eval_d_centroidal_momentum():
            self.robot.centroidalMomentum(self.q, self.dq)
            d_centroidal_momentum_matrix = self.data.dAg
            return d_centroidal_momentum_matrix

        return eval_d_centroidal_momentum

    def get_transformation(self, name, dofs=None):
        if not self.model.existFrame(name) and not name == "COM":
            raise ValueError("Transformation for %s is not available." %name)
        if name == "universe" or name == "root_joint":
            raise ValueError("Transformation for %s is not available." %name)

        def transformation():
            index = self.model.getFrameId(name)
            if dofs == "TRANSLATION":
                return self.data.oMf[index].translation
            elif dofs == "ROTATION":
                if name == "COM":
                    raise ValueError("No rotation for COM available")
                return self.data.oMf[index].rotation
            else:
                return self.data.oMf[index]

        def transformation_com():
            return self.robot.com(self.q)

        if name == "COM":
            return transformation_com
        else:
            return transformation

    def get_desired_velocity(self, goal, transformation_func, dofs=None):
        def eval_vel(delta_t):
            if dofs == "TRANSLATION":
                #print("delta_t:" , delta_t)
                return (goal - transformation_func()) / delta_t
            elif dofs is None:
                return pinocchio.log(transformation_func().inverse() * goal).vector / delta_t
            else:
                raise ValueError("Implementation for %s not available" %dofs)

        return eval_vel

    def initDisplay(self, loadModel=True):
        self.robot.initViewer(loadModel=loadModel)
        self.robot.viewer.gui.addFloor('world/floor')
        self.robot.viewer.gui.applyConfiguration('world/floor', [
            0.0, 0.0, self.floor_height,  0.0, 0.0, 0.0, 1.0])
        self.robot.viewer.gui.refresh()

    def ensureDisplay(self):
        if not hasattr(self.robot, 'viewer'):
            self.initDisplay()

    def display(self,q):
        #RobotWrapper.display(self,q)
        self.robot.display(q)
        pinocchio.updateFramePlacements(self.model,self.data)
        self.robot.viewer.gui.refresh()


class QuadrupedWrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(QuadrupedWrapper, self).__init__()

        self.effs = ["FR", "FL", "HR", "HL"]  # order is important
        self.colors = {"HL": "r", "HR": "y", "FL": "b", "FR": "g"}
        self.joints_list = ["HFE", "KFE", "ANKLE"]
        self.floor_height = 0.

        self.robot = SoloConfig.buildRobotWrapper()

        self.num_ctrl_joints = 8

        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.data
        q = pinocchio.neutral(self.robot.model)
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.set_init_config()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        self.q = pinocchio.neutral(self.robot.model)
        self.q[2] = self.floor_height

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[:] = angle
        q_dummy[::2] = -0.5 * angle
        self.q[7:] = q_dummy.reshape([self.num_ctrl_joints, 1])

        # print(self.q)
        self.set_configuration(self.q)


class Quadruped12Wrapper(BasicRobotWrapper):

    def __init__(self, q=None):
        super(Quadruped12Wrapper, self).__init__()

        self.effs = ["FR", "FL", "HR", "HL"]  # order is important
        self.colors = {"HL": "r", "HR": "y", "FL": "b", "FR": "g"}
        self.joints_list = ["HAA", "HFE", "KFE", "ANKLE"]
        self.floor_height = 0.

        self.robot = Solo12Config.buildRobotWrapper()

        self.num_ctrl_joints = 12

        # Create data again after setting frames
        self.model = self.robot.model
        self.data = self.robot.data
        q = pinocchio.neutral(self.robot.model)
        if not q is None:
            self.set_configuration(q)
        else:
            self.q = None
        self.M_com = None
        self.mass = sum([i.mass for i in self.model.inertias[1:]])
        self.set_init_config()

    def set_init_config(self):
        model = self.model
        data = self.data
        NQ = model.nq
        NV = model.nv
        self.q, self.dq, self.ddq, tau = zero(NQ), zero(NV), zero(NV), zero(NV)

        self.q = self.robot.model.neutralConfiguration.copy()
        self.q[2] = self.floor_height

        # Set initial configuration
        angle = np.deg2rad(60.0)
        q_dummy = np.zeros(self.num_ctrl_joints)
        q_dummy[2::3] = angle
        q_dummy[1:3] = -0.5 * angle

        self.q[7:] = np.reshape(q_dummy, (self.num_ctrl_joints, 1))

        # print(self.q)
        self.set_configuration(self.q)


############################ For debugging ##########################################

# robot = QuadrupedWrapper()
