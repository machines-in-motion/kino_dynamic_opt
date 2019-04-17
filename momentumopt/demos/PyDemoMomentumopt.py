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
from momentumopt.kinoptpy.kinematics_optimizer import KinematicsOptimizer, create_time_vector
from momentumexe.motion_execution import MotionExecutor
from momentumopt.kinoptpy.create_data_file import create_file, create_lqr_impedance

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

class MotionPlanner():

    def __init__(self, cfg_file, KinOpt=KinematicsOptimizer):
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

        'optimize motion'
        self.dyn_optimizer = DynamicsOptimizer()
        self.dyn_optimizer.initialize(self.planner_setting)

        'Kinematics Optimizer'
        self.kin_optimizer = KinOpt()
        self.kin_optimizer.initialize(self.planner_setting)

        self.dynamics_feedback = None

    def optimize_dynamics(self, kd_iter):
        print("DynOpt", kd_iter)
        start = time.time()
        self.dyn_optimizer.optimize(self.ini_state, self.contact_plan,
                                    self.kin_optimizer.kinematics_sequence, kd_iter > 0)
        print("Dynopt - " , time.time() -start)

    def optimize_kinematics(self, kd_iter, plotting=False):
        print("KinOpt", kd_iter)
        start = time.time()
        self.kin_optimizer.optimize(self.ini_state, self.contact_plan.contactSequence(),
                                    self.dyn_optimizer.dynamicsSequence(), plotting=plotting)
        print("kinopt - ", time.time() - start)

    def optimize_dynamics_feedback(self):
        # 'define dynamics feedback controller'
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
        self.dynamics_feedback = DynamicsFeedback()
        self.dynamics_feedback.initialize(self.dynlqr_setting, self.planner_setting)
        self.dynamics_feedback.optimize(self.ini_state, self.dyn_optimizer.dynamicsSequence())

    def plot_centroidal(self):
        fig, axes = plt.subplots(3, 1, figsize=(6, 8), sharex=True)

        dynseq = self.dyn_optimizer.dynamicsSequence()
        kinseq = self.kin_optimizer.kinematics_sequence

        for i, (ax, prop) in enumerate(zip(axes, ['com', 'lmom', 'amom'])):
            data_dyn = np.array([getattr(ds, prop) for ds in dynseq.dynamics_states])
            data_kin = np.array([getattr(ds, prop) for ds in kinseq.kinematics_states])

            for dyn, kin, label in zip(data_dyn.T, data_kin.T, ['{}_{}'.format(prop, d) for d in ['x', 'y', 'z']]):
                line = ax.plot(dyn, label=label, alpha=0.75)[0]
                ax.plot(kin, '--', color=line.get_color())[0]

            ax.legend()
            ax.grid(True)

        fig.suptitle('Centroidal info for dyn (-) and kin (--)')
        fig.tight_layout(rect=[0, 0, 1., 0.95])
        plt.show()

        return fig, axes

    def replay_kinematics(self, start=0, end=None):
        self.kin_optimizer.robot.ensureDisplay()
        for ks in self.kin_optimizer.kinematics_sequence.kinematics_states[start:end]:
            q = ks.robot_posture.generalized_joint_positions
            self.kin_optimizer.robot.display(np.matrix(q).T)
            time.sleep(self.kin_optimizer.dt)

    def save_files(self):
        time_vector = create_time_vector(self.dyn_optimizer.dynamicsSequence())
        create_file(time_vector,
                self.kin_optimizer.kinematics_sequence,
                self.dyn_optimizer.dynamicsSequence(),
                self.dynamics_feedback,
                self.planner_setting.get(PlannerDoubleParam_RobotWeight))

        # create_trajectory_file_impedance(time_vector,
        #         self.kin_optimizer.motion_eff,
        #         self.kin_optimizer.kinematics_sequence)

        create_lqr_impedance(time_vector,
                             self.kin_optimizer.motion_eff,
                             self.kin_optimizer.kinematics_sequence,
                             self.dyn_optimizer.dynamicsSequence(),
                             self.dynamics_feedback,
                             self.planner_setting.get(PlannerDoubleParam_RobotWeight))

    def time_vector(self):
        return create_time_vector(self.dyn_optimizer.dynamicsSequence())

    def optimize_motion(self):
        dyn_optimizer = self.dyn_optimizer
        kin_optimizer = self.kin_optimizer

        self.optimize_dynamics(0)
        for kd_iter in range(0, self.planner_setting.get(PlannerIntParam_KinDynIterations)):
            self.optimize_kinematics(kd_iter + 1, plotting=False)
            self.optimize_dynamics(kd_iter + 1)

        optimized_kin_plan = kin_optimizer.kinematics_sequence
        optimized_dyn_plan = dyn_optimizer.dynamicsSequence()

        time_vector = create_time_vector(dyn_optimizer.dynamicsSequence())
        self.optimize_dynamics_feedback()
        return optimized_kin_plan, kin_optimizer.motion_eff, \
                optimized_dyn_plan, self.dynamics_feedback, \
                self.planner_setting, time_vector


'Main function for optimization demo'
def main(argv):

    cfg_file = ''
    kinopt = KinematicsOptimizer
    use_momentum_kin_opt = False
    try:
        opts, args = getopt.getopt(argv,"hi:m",["ifile="])
    except getopt.GetoptError:
        print ('PyDemoMomentumopt.py -i <path_to_datafile>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-m"):
            use_momentum_kin_opt = True
            kinopt = MomentumKinematicsOptimizer
        elif opt in ("-i", "--ifile"):
            cfg_file = arg

    print(opts)
    print(cfg_file)

    motion_planner = MotionPlanner(cfg_file, KinOpt=kinopt)

    # TODO: Read the weights from the config file.
    if use_momentum_kin_opt:
        kin_optimizer = motion_planner.kin_optimizer
        inv_kin = kin_optimizer.inv_kin
        etg = kin_optimizer.endeff_traj_generator
        etg.z_offset = 0.1

        inv_kin.w_com_tracking[:3] = 1.
        inv_kin.w_com_tracking[3:] = 1.
        inv_kin.w_endeff_contact = 1.
        inv_kin.p_endeff_tracking = 1.
        inv_kin.p_com_tracking = 1.
        kin_optimizer.reg_orientation = .05

    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector = motion_planner.optimize_motion()
    # for i in range(len(time_vector)):
    #     print "\n t:",time_vector[i],"\n"
    #     print dynamics_feedback.forceGain(0)

        # motion_planner.plot_centroidal()
    # Create configuration and velocity file from motion plan for dynamic graph
    motion_planner.save_files()
    simulation = False

    plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)

    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
