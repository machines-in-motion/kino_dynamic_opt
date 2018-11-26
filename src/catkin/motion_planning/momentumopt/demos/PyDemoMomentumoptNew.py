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
from pysolverlqr import *
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
from quadruped.PyQuadrupedRobot import QuadrupedWrapper
import os, sys, getopt, time, numpy as np, pinocchio as pin

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.momentumopt.kinoptpy.kinematics_optimizer import KinematicsOptimizer, create_time_vector

'Kinematics Interface using Pinocchio'
class PinocchioKinematicsInterface(KinematicsInterface):
    def __init__(self):
        KinematicsInterface.__init__(self)

    def initialize(self, planner_setting):
        self.eff_names = {0: "BR_END", 1: "BL_END", 2: "FR_END", 3: "FL_END"}
        urdf = str(os.path.dirname(os.path.abspath(__file__)) + '/quadruped/quadruped.urdf')

        self.robot = QuadrupedWrapper(urdf)
        self.robot.initDisplay(loadModel=True)
        self.robot.viewer.gui.applyConfiguration('world/floor',[0.0, 0.0, -0.32,  0.0, 0.0, 0.0, 1.0])

    def displayPosture(self, kin_state, wait_time):
        self.q = np.matrix(np.squeeze(np.asarray(kin_state.robot_posture.generalized_joint_positions()))).transpose()
        self.robot.display(self.q)
        time.sleep(wait_time)

    def updateJacobians(self, kin_state):
        'Generalized joint positions and velocities'
        self.q = np.matrix(np.squeeze(np.asarray(kin_state.robot_posture.generalized_joint_positions()))).transpose()
        self.dq = np.matrix(np.squeeze(np.asarray(kin_state.robot_velocity.generalized_joint_velocities))).transpose()

        'Update of jacobians'
        self.robot.computeJointJacobians(self.q);
        self.robot.framesForwardKinematics(self.q)
        for eff_id in range(0, len(self.eff_names)):
            self.endeffector_jacobians[eff_id] = self.robot.getFrameJacobian(self.robot.model.getFrameId(self.eff_names[eff_id]), pin.ReferenceFrame.LOCAL)

        self.robot.centroidalMomentum(self.q, self.dq)
        self.centroidal_momentum_matrix = self.robot.data.Ag

        'Update of kinematics state'
        kin_state.com = self.robot.com(self.q)
        kin_state.lmom = self.robot.data.hg.vector[:3]
        kin_state.amom = self.robot.data.hg.vector[3:]

        for eff_id in range(0, len(self.eff_names)):
            kin_state.endeffector_positions[eff_id] = self.robot.data.oMf[self.robot.model.getFrameId(self.eff_names[eff_id])].translation

        return kin_state

'Main function for optimization demo'
def main(argv):

    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print ('PyDemoMomentumopt.py -i <path_to_datafile>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg

    'define problem configuration'
    planner_setting = PlannerSetting()
    planner_setting.initialize(cfg_file)

    dynlqr_setting = SolverLqrSetting()
    dynlqr_setting.initialize(cfg_file, "solverlqr_dynamics")

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

    'create instances of optimizers and initialize'
    dyn_optimizer = DynamicsOptimizer()
    kin_optimizer = KinematicsOptimizer()
    kin_interface = PinocchioKinematicsInterface()

    dyn_optimizer.initialize(planner_setting)
    kin_optimizer.initialize(planner_setting, kin_interface)

    'optimize motion'
    print("DynOpt", 0)
    dyn_optimizer.optimize(ini_state, contact_plan, kin_sequence, False)
    for kd_iter in range(0, planner_setting.get(PlannerIntParam_KinDynIterations)):
        print("KinOpt", kd_iter+1)
        kin_optimizer.optimize(ini_state, contact_plan, dyn_optimizer.dynamicsSequence(), kd_iter>0)
        print("DynOpt", kd_iter+1)
        dyn_optimizer.optimize(ini_state, contact_plan, kin_optimizer.kinematicsSequence(), True)    

#     'Kinematics Optimizer'
#     print("KinOpt", 0)
#     kin_optimizer = KinematicsOptimizer()
#     kin_optimizer.initialize(planner_setting)
#     kin_optimizer.optimize(ini_state, contact_plan.contactSequence(), dyn_optimizer.dynamicsSequence())
#     dyn_optimizer.optimize(ini_state, contact_plan, kin_optimizer.kinematics_sequence, True)


    # print dyn_optimizer.solveTime()
    # print dyn_optimizer.dynamicsSequence().dynamics_states[planner_setting.get(PlannerIntParam_NumTimesteps)-1]
    # print contact_plan.contactSequence().contact_states(0)[0].position

    'define dynamics feedback'
    dynamics_feedback = DynamicsFeedback()
    dynamics_feedback.initialize(dynlqr_setting, planner_setting)
    dynamics_feedback.optimize(ini_state, dyn_optimizer.dynamicsSequence())
    #Access feedback gains using: dynamics_feedback.forceGain(time_id)
    

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
