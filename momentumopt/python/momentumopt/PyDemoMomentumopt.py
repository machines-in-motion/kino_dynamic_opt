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

from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer
from momentumexe.motion_execution import MotionExecutor
from momentumopt.kinoptpy.create_data_file import create_file, create_qp_files, create_lqr_files

from momentumopt.utilities.motion_planner import MotionPlanner

import matplotlib.pyplot as plt

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

def parse_arguments(argv):
    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:m",["ifile="])
    except getopt.GetoptError:
        print ('PyDemoMomentumopt.py -i <path_to_datafile>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg
    
    print(opts)
    print(cfg_file)

    if not os.path.exists(cfg_file):
        raise RuntimeError("The config file " + cfg_file + " does not exist.")
    
    return cfg_file

def optimize_the_motion(cfg_file, plot_com_motion=True):
    """
    Optimize the motion using the kino-dyn optimizer.
    For the dynamics we use the centroidal dynamics solver from this package.
    Fro the Kinematics we use a python written kinematics solver.
    """
    # create the planner
    motion_planner = MotionPlanner(cfg_file)

    # load all the parameters of the planner
    etg = motion_planner.kin_optimizer.endeff_traj_generator
    etg.z_offset = motion_planner.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ)
    motion_planner.kin_optimizer.inv_kin.w_lin_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightLinMomentumTracking)
    motion_planner.kin_optimizer.inv_kin.w_ang_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightAngMomentumTracking)
    motion_planner.kin_optimizer.inv_kin.w_endeff_contact = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffContact)
    motion_planner.kin_optimizer.inv_kin.w_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffTracking)
    motion_planner.kin_optimizer.inv_kin.p_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainEndEffTracking)
    motion_planner.kin_optimizer.inv_kin.p_com_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainComTracking)
    motion_planner.kin_optimizer.inv_kin.w_joint_regularization = motion_planner.planner_setting.get(PlannerDoubleParam_WeightJointReg)
    motion_planner.kin_optimizer.reg_orientation = motion_planner.planner_setting.get(PlannerDoubleParam_PGainOrientationTracking)

    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector = \
      motion_planner.optimize_motion(plot_com_motion)

    # Optimize the dynamic and kinematic motion.
    return optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
           dynamics_feedback, planner_setting, time_vector, motion_planner


def main(argv):
    """
    Main function for optimization demo
    """
    cfg_file = parse_arguments(argv)
    
    (optimized_kin_plan,
     optimized_motion_eff,
     optimized_dyn_plan,
     dynamics_feedback,
     planner_setting,
     time_vector,
     motion_planner) = optimize_the_motion(cfg_file)
    
    display = True
    if(display): # Display the Center of mass motion
        motion_planner.plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
        # for i in range(len(time_vector)):
        #     print "\n t:",time_vector[i],"\n"
        #     print dynamics_feedback.forceGain(i)
        # motion_planner.plot_centroidal()

    # Create configuration and velocity file from motion plan for dynamic graph
    try:
        motion_planner.replay_kinematics()
    except:
        "gepetto not initialized..."

    # Dump the computed trajectory in a files (should follow the dynamic graph format)
    motion_planner.save_files()

    if(display): # plot trajectories
        motion_planner.plot_foot_traj()
        motion_planner.plot_joint_trajecory()
        motion_planner.plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
        #motion_planner.plot_base_trajecory()

    # Potentially simulate the motion
    simulation = False
    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
