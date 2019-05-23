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


def plot_com_motion(dynamics_states, kinematics_states):
    fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharex=True)
    axes = np.array(axes)

    def states_to_vec(states):
        com = np.vstack([s.com for s in states])
        lmom = np.vstack([s.lmom for s in states])
        amom = np.vstack([s.amom for s in states])
        return com, lmom, amom


    for i, (title, dyn, kin) in enumerate(zip(
        ['com', 'lmom', 'amom'],
        states_to_vec(dynamics_states),
        states_to_vec(kinematics_states))):

        axes[0, i].set_title(title)

        for j in range(3):
            axes[j, i].plot(dyn[:, j], label='desired')
            axes[j, i].plot(kin[:, j], label='actual')

    [ax.grid(True) for ax in axes.reshape(-1)]

    for i, label in enumerate(['x', 'y', 'z']):
        axes[i, 0].set_ylabel(label + ' [m]')
        axes[2, i].set_xlabel('time steps [5ms]')

    axes[0, 2].legend()

    plt.show()

'Main function for optimization demo'
def main(argv):

    cfg_file = ''
    kinopt = KinematicsOptimizer
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

    motion_planner = MotionPlanner(cfg_file)

    # TODO: Read the kinematic weights from the config file.
    kin_optimizer = motion_planner.kin_optimizer
    inv_kin = kin_optimizer.inv_kin
    etg = kin_optimizer.endeff_traj_generator
    etg.z_offset = motion_planner.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ)

    inv_kin.w_lin_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightLinMomentumTracking)
    inv_kin.w_ang_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightAngMomentumTracking)
    inv_kin.w_endeff_contact = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffContact)
    inv_kin.w_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffTracking)
    inv_kin.p_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainEndEffTracking)
    inv_kin.p_com_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainComTracking)
    inv_kin.w_joint_regularization = motion_planner.planner_setting.get(PlannerDoubleParam_WeightJointReg)
    kin_optimizer.reg_orientation = motion_planner.planner_setting.get(PlannerDoubleParam_PGainOrientationTracking)
    print inv_kin.w_joint_regularization

    # Optimize the dynamic and kinematic motion.
    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector = motion_planner.optimize_motion()
    #for i in range(len(time_vector)):
    #    print "\n t:",time_vector[i],"\n"
    #    print dynamics_feedback.forceGain(i)
        # motion_planner.plot_centroidal()
    # Create configuration and velocity file from motion plan for dynamic graph
    try:
        motion_planner.replay_kinematics()
    except:
        "gepeto not initialized..."
    motion_planner.save_files()
    simulation = False
    motion_planner.plot_foot_traj()
    plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
    #motion_planner.plot_base_trajecory()

    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)


    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
