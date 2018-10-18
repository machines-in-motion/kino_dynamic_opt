#!/usr/bin/python

'''
 Copyright [2017] Max Planck Society. All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

from pysolver import *
from pymomentum import *
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import os, sys, getopt, numpy as np, pinocchio as pin

'Kinematics Interface using Pinocchio'
class PinocchioKinematicsInterface(KinematicsInterface):
    def __init__(self):
        KinematicsInterface.__init__(self)

    def initialize(self, planner_setting):
        urdf = str(os.path.dirname(os.path.abspath(__file__)) + '/quadruped/quadruped.urdf')
        self.robot = RobotWrapper(urdf, root_joint=pin.JointModelFreeFlyer())

    def updateJacobians(self, kin_state):
        'Generalized joint positions and velocities'
        self.q = np.matrix(np.squeeze(np.asarray(kin_state.robot_posture.generalized_joint_positions()))).transpose()
        self.dq = np.matrix(np.squeeze(np.asarray(kin_state.robot_velocity.generalized_joint_velocities))).transpose()

        'Update of jacobians'
        self.robot.framesForwardKinematics(self.q)
        self.endeffector_jacobians[0] = self.robot.getFrameJacobian(self.robot.model.getFrameId("BL_END"), pin.ReferenceFrame.WORLD)
        self.endeffector_jacobians[1] = self.robot.getFrameJacobian(self.robot.model.getFrameId("FL_END"), pin.ReferenceFrame.WORLD)
        self.endeffector_jacobians[2] = self.robot.getFrameJacobian(self.robot.model.getFrameId("BR_END"), pin.ReferenceFrame.WORLD)
        self.endeffector_jacobians[3] = self.robot.getFrameJacobian(self.robot.model.getFrameId("FR_END"), pin.ReferenceFrame.WORLD)

        pin.ccrba(self.robot.model, self.robot.data, self.q, self.dq)
        self.centroidal_momentum_matrix = self.robot.data.Ag

        'Update of kinematics state'
        kin_state.com = self.robot.com(self.q)
        kin_state.lmom = self.robot.data.hg.vector[:3]
        kin_state.amom = self.robot.data.hg.vector[3:]

        kin_state.endeffector_positions[0] = self.robot.data.oMf[self.robot.model.getFrameId("BL_END")].translation
        kin_state.endeffector_positions[1] = self.robot.data.oMf[self.robot.model.getFrameId("FL_END")].translation
        kin_state.endeffector_positions[2] = self.robot.data.oMf[self.robot.model.getFrameId("BR_END")].translation
        kin_state.endeffector_positions[3] = self.robot.data.oMf[self.robot.model.getFrameId("FR_END")].translation

        return kin_state


'Main function for optimization demo'
def main(argv):
    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print 'PyDemoMomentumopt.py -i <path_to_datafile>'
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print 'PyDemoMomentumopt.py -i <path_to_datafile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg

    'define problem configuration'
    planner_setting = PlannerSetting()
    planner_setting.initialize(cfg_file)
 
    'define robot initial state'
    ini_state = DynamicsState()
    ini_state.fillInitialRobotState(cfg_file)
 
    'define reference dynamic sequence'
    kin_sequence = KinematicsSequence()
    kin_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                        planner_setting.get(PlannerIntParam_NumDofs))

    'define terrain description'
    terrain_description = TerrainDescription()
    terrain_description.loadFromFile(planner_setting.get(PlannerStringParam_ConfigFile))
 
    'define contact plan'
    contact_plan = ContactPlanFromFile()
    contact_plan.initialize(planner_setting)
    contact_plan.optimize(ini_state, terrain_description)
      
    'optimize motion'
    dyn_optimizer = DynamicsOptimizer()
    dyn_optimizer.initialize(planner_setting)
    dyn_optimizer.optimize(ini_state, contact_plan, kin_sequence)
  
    #print dyn_optimizer.solveTime()
    #print dyn_optimizer.dynamicsSequence().dynamics_states[planner_setting.get(PlannerIntParam_NumTimesteps)-1]
     
################################################################
################################################################
    'Kinematics Interface'
    kin_interface = PinocchioKinematicsInterface()
#    kin_interface.initialize()

    'Kinematics Optimizer'
    kin_optimizer = KinematicsOptimizer()
    kin_optimizer.initialize(planner_setting, kin_interface)
################################################################
################################################################
    

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
