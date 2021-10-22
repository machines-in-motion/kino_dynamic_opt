## This file contains the kinematics action model for DDP based 
## inverse kinematics
## Author : Avadesh Meduri
## Date : 12/02/2021

import numpy as np
import pinocchio
import crocoddyl

class DifferentialFwdKinematics(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, state, actuation, costModel):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, state, state.nv, costModel.nr)
        self.costs = costModel
        self.enable_force = True
        self.armature = np.zeros(0)
        self.no_states = self.state.nq + self.state.nv

    def calc(self, data, x, u=None):
        if u is None:
            u = self.unone
        q, v = x[:self.state.nq], x[-self.state.nv:]
                    
        pinocchio.forwardKinematics(self.state.pinocchio, data.pinocchio, q, v)
        pinocchio.updateFramePlacements(self.state.pinocchio, data.pinocchio)
        pinocchio.centerOfMass(self.state.pinocchio, data.pinocchio, q, v)
        pinocchio.computeCentroidalMomentum(self.state.pinocchio, data.pinocchio)
        data.xout = u
        # print(np.shape(u))
        self.costs.calc(data.costs, x, u)
        data.cost = data.costs.cost

    def calcDiff(self, data, x, u=None):
        q, v = x[:self.state.nq], x[-self.state.nv:]
        if u is None:
            u = self.unone
        if True:
            self.calc(data, x, u)
        
        # u_a = np.concatenate((np.zeros(6), u))
        pinocchio.computeForwardKinematicsDerivatives(self.state.pinocchio, data.pinocchio, q, v, u)
        pinocchio.jacobianCenterOfMass(self.state.pinocchio, data.pinocchio)
        pinocchio.computeCentroidalDynamicsDerivatives(self.state.pinocchio, data.pinocchio, q, v, u)
        data.Fx = self.dynamics_x(q,v,u)
        data.Fu = self.dynamics_u(q,v,u)
        # Computing the cost derivatives
        self.costs.calcDiff(data.costs, x, u)
    
    def dynamics_x(self, q, v, u):
        '''
        Returns the derivative of the dynamics with respect to states
        Input:
            q : joint position 
            v : joint velocity
            u : torque applied at the end of manipulator
        '''
        A_lin = np.zeros((int(self.no_states/2),self.no_states))
        
        return A_lin
    
    def dynamics_u(self, x, u, dt):
        ''' 
        Returns the derivative of the dynamics with respect to torques
        Input:
            q : joint position 
            v : joint velocity
            u : torque applied at the end of manipulator
        '''
        B_lin = np.eye(int(self.no_states/2))

        return B_lin
    
    def set_armature(self, armature):
        if armature.size is not self.state.nv:
            print('The armature dimension is wrong, we cannot set it.')
        else:
            self.enable_force = False
            self.armature = armature.T

    def createData(self):
        data = crocoddyl.DifferentialActionModelAbstract.createData(self)
        data.pinocchio = pinocchio.Data(self.state.pinocchio)
        data.costs = self.costs.createData(crocoddyl.DataCollectorMultibody(data.pinocchio))
        data.costs.shareMemory(data) # this allows us to share the memory of cost-terms of action model
        return data