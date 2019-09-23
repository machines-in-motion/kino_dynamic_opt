import os
import pdb
import numpy as np
from time import sleep

import pinocchio as pin
from pymomentum import *
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper

class PinocchioKinematicsInterface(KinematicsInterface):

    def __init__(self):
        KinematicsInterface.__init__(self)

    def initialize(self, planner_setting):
        package_dirs = [os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))]
        urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/urdf/quadruped.urdf')
        self.robot = RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs, root_joint=pin.JointModelFreeFlyer())
        self.robot.data = self.robot.model.createData()

        self.initDisplay(loadModel=True)
        self.robot.viewer.gui.addFloor('world/floor')
        self.z_floor = planner_setting.get(PlannerDoubleParam_FloorHeight)
        self.robot.viewer.gui.applyConfiguration('world/floor',[0.0, 0.0, self.z_floor,  0.0, 0.0, 0.0, 1.0])
        self.robot.viewer.gui.refresh()

        self.robot.q = self.robot.model.referenceConfigurations.copy()
        self.robot.dq = zero(self.robot.model.nv)
        self.robot.q[7:] = np.transpose(np.matrix(planner_setting.get(PlannerVectorParam_KinematicDefaultJointPositions)))
        pin.forwardKinematics(self.robot.model, self.robot.data, self.robot.q)
        self.robot.display(self.robot.q)

    def displayPosture(self, kin_state, wait_time):
        self.display(np.transpose(np.matrix(kin_state.robot_posture.generalized_joint_positions())))
        sleep(wait_time)

    def updateJacobiansAndState(self, kin_state, dt):
        jnt_names = ["HFE", "KFE", "END"]
        eff_names = ["BR", "BL", "FR", "FL"]

        self.robot.q = np.transpose(np.matrix(kin_state.robot_posture.generalized_joint_positions()))
        self.robot.dq = np.transpose(np.matrix(kin_state.robot_velocity.generalized_joint_velocities))
        self.robot.ddq = np.transpose(np.matrix(kin_state.robot_acceleration.generalized_joint_accelerations))
        self.robot.centroidalMomentumVariation(self.robot.q, self.robot.dq, self.robot.ddq)

        'Update objective function'
        kin_state.com = self.robot.com(self.robot.q)
        kin_state.lmom = self.robot.data.hg.vector[:3]
        kin_state.amom = self.robot.data.hg.vector[3:]
        self.centroidal_momentum_matrix = self.robot.data.Ag
        self.centroidal_momentum_matrix_variation = self.robot.data.dAg
        self.center_of_mass_jacobian = self.robot.Jcom(self.robot.q)
        for eff_id in range(0,len(eff_names)):
            nameToIndexMap = self.robot.model.getFrameId(eff_names[eff_id] + "_" + jnt_names[-1])
            self.endeffector_jacobians[eff_id] = pin.frameJacobian(self.robot.model, self.robot.data, self.robot.q, nameToIndexMap, pin.ReferenceFrame.LOCAL)[0:3, :]
            kin_state.endeffector_positions[eff_id] = np.squeeze(np.array(self.robot.data.oMf[nameToIndexMap].translation))

        'Update constraints'
        num_constraints = len(eff_names) * len(jnt_names)
        G = np.zeros((num_constraints, self.robot.nv))
        h = np.zeros((num_constraints))
        for eff_id in range(0,len(eff_names)):
            for jnt_id in range(0,len(jnt_names)):
                nameToIndexMap = self.robot.model.getFrameId(eff_names[eff_id] + "_" + jnt_names[jnt_id])
                G[eff_id*len(jnt_names)+jnt_id, :] = np.squeeze(np.asarray(pin.frameJacobian(self.robot.model, self.robot.data, self.robot.q, nameToIndexMap, pin.ReferenceFrame.LOCAL)[2, :]))
                h[eff_id*len(jnt_names)+jnt_id] = self.robot.data.oMf[nameToIndexMap].translation[-1][0, 0]
        self.constraintsMatrix = - G*dt
        self.constraintsVector = h - self.z_floor

        return kin_state

    def integratePosture(self, kin_state, dt):
        self.robot.q = np.transpose(np.matrix(kin_state.robot_posture.generalized_joint_positions()))
        self.robot.dq = np.transpose(np.matrix(kin_state.robot_velocity.generalized_joint_velocities))
        self.robot.q = pin.integrate(self.robot.model, self.robot.q, self.robot.dq * dt)

        kin_state.robot_posture.base_position = np.squeeze(np.asarray(self.robot.q[0:3]))
        kin_state.robot_posture.joint_positions = np.squeeze(np.asarray(self.robot.q[7:]))
        kin_state.robot_posture.base_orientation = np.squeeze(np.asarray(self.robot.q[3:7]))

        return kin_state

    def differentiatePostures(self, ini_state, end_state, dt):
        q_ini = np.transpose(np.matrix(ini_state.robot_posture.generalized_joint_positions()))
        q_end = np.transpose(np.matrix(end_state.robot_posture.generalized_joint_positions()))
        q_dot = pin.difference(self.robot.model, q_ini, q_end) / dt
        end_state.robot_velocity.generalized_joint_velocities = np.squeeze(np.asarray(q_dot))

        return end_state

    def logarithmicMap(self, quat_wxyz):
        w,x,y,z = quat_wxyz
        return pin.log(pin.Quaternion(w,x,y,z).matrix());

    def initDisplay(self,loadModel):
        self.robot.initDisplay(loadModel=loadModel)

        # manual display of robot parts
        # self.robot.viewer.gui.addBox     ('world/basis'    , .10, .20, .025,[1.0, 1.0, 1.0, 1.0])
        # self.robot.viewer.gui.addCylinder('world/blfemoral', .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/blshank'  , .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/brfemoral', .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/brshank'  , .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/flfemoral', .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/flshank'  , .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/frfemoral', .02, .16,      [0.4, 0.4, 0.4, 1.0])
        # self.robot.viewer.gui.addCylinder('world/frshank'  , .02, .16,      [0.4, 0.4, 0.4, 1.0])
        #
        # self.robot.viewer.gui.addSphere('world/blhip' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/blknee', .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/bleff' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/brhip' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/brknee', .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/breff' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/flhip' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/flknee', .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/fleff' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/frhip' , .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/frknee', .03, [1.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.addSphere('world/freff' , .03, [1.0, 0.0, 0.0, 1.0])
        #
        # self.robot.viewer.gui.setStaticTransform('world/blfemoral', [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/blshank'  , [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/brfemoral', [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/brshank'  , [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/flfemoral', [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/flshank'  , [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/frfemoral', [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])
        # self.robot.viewer.gui.setStaticTransform('world/frshank'  , [0.0, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0])

        self.robot.viewer.gui.refresh()

    def display(self,q):
        self.robot.display(q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        # self.robot.viewer.gui.applyConfiguration('world/basis'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("base_link")]))
        # self.robot.viewer.gui.applyConfiguration('world/blfemoral', se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BL_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/blshank'  , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BL_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/brfemoral', se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BR_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/brshank'  , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BR_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/flfemoral', se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FL_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/flshank'  , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FL_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/frfemoral', se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FR_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/frshank'  , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FR_KFE")]))
        #
        # self.robot.viewer.gui.applyConfiguration('world/blhip'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BL_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/blknee'   , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BL_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/bleff'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BL_END")]))
        # self.robot.viewer.gui.applyConfiguration('world/brhip'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BR_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/brknee'   , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BR_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/breff'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("BR_END")]))
        # self.robot.viewer.gui.applyConfiguration('world/flhip'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FL_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/flknee'   , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FL_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/fleff'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FL_END")]))
        # self.robot.viewer.gui.applyConfiguration('world/frhip'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FR_HFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/frknee'   , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FR_KFE")]))
        # self.robot.viewer.gui.applyConfiguration('world/freff'    , se3ToXYZQUAT(self.robot.data.oMf[self.robot.model.getFrameId("FR_END")]))

        self.robot.viewer.gui.refresh()
