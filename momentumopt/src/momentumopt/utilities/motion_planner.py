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

import numpy as np

from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer
from momentumopt.kinoptpy.create_data_file import create_file, create_qp_files, create_lqr_files

import matplotlib.pyplot as plt

def create_time_vector(dynamics_sequence):
    num_time_steps = len(dynamics_sequence.dynamics_states)
    # Create time vector
    time = np.zeros((num_time_steps))
    time[0] = dynamics_sequence.dynamics_states[0].dt
    for i in range(num_time_steps - 1):
        time[i + 1] = time[i] + dynamics_sequence.dynamics_states[i].dt

    return time

class MotionPlanner():

    def __init__(self, cfg_file, KinOpt=MomentumKinematicsOptimizer):
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

    def plot_base_trajecory(self, start=0, end=None):
        q_app = np.zeros([1,self.kin_optimizer.robot.model.nq])
        for ks in self.kin_optimizer.kinematics_sequence.kinematics_states[start:end]:
            q = ks.robot_posture.generalized_joint_positions
            q_app = np.append(q_app,q.reshape(1,len(q)),axis=0)
        fig, ax = plt.subplots(3,1)
        label = ["base_x","base_y","base_z"]
        for i in range(3):
            ax[i].plot(q_app[1:end,i], label = label[i])
            ax[i].set_ylabel("m")
            ax[i].set_xlabel("millisec")
            ax[i].legend()
            ax[i].grid(True)
        plt.show()


    def save_files(self):
        time_vector = create_time_vector(self.dyn_optimizer.dynamicsSequence())
        create_file(time_vector,
                self.kin_optimizer.kinematics_sequence,
                self.dyn_optimizer.dynamicsSequence(),
                self.dynamics_feedback,
                self.planner_setting.get(PlannerDoubleParam_RobotWeight))

        create_qp_files(time_vector,
                             self.kin_optimizer.motion_eff,
                             self.kin_optimizer.kinematics_sequence,
                             self.dyn_optimizer.dynamicsSequence(),
                             self.dynamics_feedback,
                             self.planner_setting.get(PlannerDoubleParam_RobotWeight))

        # create_lqr_files(time_vector,
        #                      self.kin_optimizer.motion_eff,
        #                      self.kin_optimizer.kinematics_sequence,
        #                      self.dyn_optimizer.dynamicsSequence(),
        #                      self.dynamics_feedback,
        #                      self.planner_setting.get(PlannerDoubleParam_RobotWeight))

    def time_vector(self):
        return create_time_vector(self.dyn_optimizer.dynamicsSequence())

    def optimize_motion(self):
        dyn_optimizer = self.dyn_optimizer
        kin_optimizer = self.kin_optimizer

        self.optimize_dynamics(0)
        for kd_iter in range(0, self.planner_setting.get(PlannerIntParam_KinDynIterations)):
            self.optimize_kinematics(kd_iter + 1, plotting=False)
            self.optimize_dynamics(kd_iter + 1)
            #optimized_kin_plan = kin_optimizer.kinematics_sequence
            #optimized_dyn_plan = dyn_optimizer.dynamicsSequence()
            #plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)

        optimized_kin_plan = kin_optimizer.kinematics_sequence
        optimized_dyn_plan = dyn_optimizer.dynamicsSequence()

        time_vector = create_time_vector(dyn_optimizer.dynamicsSequence())
        self.optimize_dynamics_feedback()
        return optimized_kin_plan, kin_optimizer.motion_eff, \
                optimized_dyn_plan, self.dynamics_feedback, \
                self.planner_setting, time_vector
