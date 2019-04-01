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
import time
from pysolver import *
from pymomentum import *
from pysolverlqr import *
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import os, sys, getopt, numpy as np, pinocchio as pin

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.momentumopt.kinoptpy.kinematics_optimizer import KinematicsOptimizer, create_time_vector
from src.momentumexe.motion_execution import MotionExecutor
from src.momentumopt.kinoptpy.create_data_file import create_file, create_trajectory_file_impedance

'Kinematics Interface using Pinocchio'
class PinocchioKinematicsInterface(KinematicsInterface):
    def __init__(self):
        KinematicsInterface.__init__(self)

    def initialize(self, planner_setting):
        urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/urdf/quadruped.urdf')
        #print("urdf_path:", urdf)
        self.robot = RobotWrapper(urdf, root_joint=pin.JointModelFreeFlyer())
        self.eff_names = {0: "BL_END", 1: "FL_END", 2: "BR_END", 3: "FR_END"}

    def updateJacobians(self, kin_state):
        'Generalized joint positions and velocities'
        self.q = np.matrix(np.squeeze(np.asarray(kin_state.robot_posture.generalized_joint_positions()))).transpose()
        self.dq = np.matrix(np.squeeze(np.asarray(kin_state.robot_velocity.generalized_joint_velocities))).transpose()

        'Update of jacobians'
        self.robot.computeJointJacobians(self.q);
        self.robot.framesForwardKinematics(self.q)
        for eff_id in range(0, len(self.eff_names)):
            self.endeffector_jacobians[eff_id] = self.robot.getFrameJacobian(self.robot.model.getFrameId(self.eff_names[eff_id]), pin.ReferenceFrame.WORLD)

        self.robot.centroidalMomentum(self.q, self.dq)
        self.centroidal_momentum_matrix = self.robot.data.Ag

        'Update of kinematics state'
        kin_state.com = self.robot.com(self.q)
        kin_state.lmom = self.robot.data.hg.vector[:3]
        kin_state.amom = self.robot.data.hg.vector[3:]

        for eff_id in range(0, len(self.eff_names)):
            kin_state.endeffector_positions[eff_id] = self.robot.data.oMf[self.robot.model.getFrameId(self.eff_names[eff_id])].translation


class MotionPlanner():

    def __init__(self, cfg_file):
        'define problem configuration'
        self.planner_setting = PlannerSetting()
        self.planner_setting.initialize(cfg_file)

        self.dynlqr_setting = SolverLqrSetting()
        self.dynlqr_setting.initialize(cfg_file, "solverlqr_dynamics")

        'define robot initial state'
        self.ini_state = DynamicsState()
        self.ini_state.fillInitialRobotState(cfg_file)

        'define reference dynamic sequence'
        #self.kin_sequence = KinematicsSequence()
        #self.kin_sequence.resize(self.planner_setting.get(PlannerIntParam_NumTimesteps),
        #                         self.planner_setting.get(PlannerIntParam_NumDofs))

        'define terrain description'
        self.terrain_description = TerrainDescription()
        self.terrain_description.loadFromFile(self.planner_setting.get(PlannerStringParam_ConfigFile))

        'define contact plan'
        self.contact_plan = ContactPlanFromFile()
        self.contact_plan.initialize(self.planner_setting)
        self.contact_plan.optimize(self.ini_state, self.terrain_description)


    def optimize_motion(self):
        'optimize motion'
        dyn_optimizer = self.dyn_optimizer = DynamicsOptimizer()
        dyn_optimizer.initialize(self.planner_setting)

        'Kinematics Optimizer'
        kin_optimizer = self.kin_optimizer = KinematicsOptimizer()
        kin_optimizer.initialize(self.planner_setting)

        print("DynOpt", 0)
        start = time.time()
        dyn_optimizer.optimize(self.ini_state, self.contact_plan, kin_optimizer.kinematics_sequence, False)
        end = time.time()
        print("Dynopt - " ,end-start)
        for kd_iter in range(0, self.planner_setting.get(PlannerIntParam_KinDynIterations)):
            print("KinOpt", kd_iter+1)
            if kd_iter == self.planner_setting.get(PlannerIntParam_KinDynIterations) - 1:
                start = time.time()
                kin_optimizer.optimize(self.ini_state, self.contact_plan.contactSequence(), dyn_optimizer.dynamicsSequence(), plotting=True)
                end = time.time()
                print("kinopt - ", end-start)

            else:
                start = time.time()
                kin_optimizer.optimize(self.ini_state, self.contact_plan.contactSequence(), dyn_optimizer.dynamicsSequence(), plotting=True)
                end = time.time()
                print("kinopt - ", end-start)

            print("DynOpt", kd_iter+1)
            start = time.time()
            dyn_optimizer.optimize(self.ini_state, self.contact_plan, kin_optimizer.kinematics_sequence, True)
            end = time.time()
            print("dynopt - ", end-start)

        optimized_kin_plan = kin_optimizer.kinematics_sequence
        optimized_dyn_plan = dyn_optimizer.dynamicsSequence()

        optimized_motion_eff = kin_optimizer.motion_eff

        time_vector = create_time_vector(dyn_optimizer.dynamicsSequence())

        # 'define dynamics feedback controller'
        # dynamics_feedback = DynamicsFeedback()
        # dynamics_feedback.initialize(self.dynlqr_setting, self.planner_setting)
        # dynamics_feedback.optimize(self.ini_state, dyn_optimizer.dynamicsSequence())
        # '''
        # Access feedback gains using: dynamics_feedback.forceGain(time_id)
        #                             [currentCOM  - desiredCoM ]
        #   deltaForce = forceGain *  [currentLMOM - desiredLMOM]
        #                             [currentAMOM - desiredAMOM]
        #
        #  Torque = PD(q,qdot) + J^T * (plannedForce + deltaForce)
        #  Remember that plannedForce of dyn_optimizer is normalized by robot weight
        #  (self.planner_setting.get(PlannerDoubleParam_RobotWeight)),
        #  so you need to multiply it by that amount for it to work!
        #  deltaForce comes already in the right units.
        # '''

        # print dyn_optimizer.solveTime()
        # print dyn_optimizer.dynamicsSequence().dynamics_states[planner_setting.get(PlannerIntParam_NumTimesteps)-1]
        # print contact_plan.contactSequence().contact_states(0)[0].position

        return optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, self.planner_setting, time_vector


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

    print(cfg_file)

    motion_planner = MotionPlanner(cfg_file)
    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, planner_setting, time_vector = motion_planner.optimize_motion()

    # Create configuration and velocity file from motion plan for dynamic graph
    # create_file(time_vector, optimized_kin_plan, optimized_dyn_plan,
    #         motion_planner.planner_setting.get(PlannerDoubleParam_RobotWeight))
    create_trajectory_file_impedance(time_vector, optimized_motion_eff, optimized_kin_plan)
    simulation = False

    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
