import os
import numpy as np
import pdb

import pinocchio as se3
from pinocchio.utils import *
from pymomentum import *

from src.momentumopt.kinoptpy.min_jerk_traj import generate_eff_traj
from src.quadruped.quadruped_wrapper import QuadrupedWrapper
from src.momentumopt.kinoptpy.inv_kin import InverseKinematics
from src.momentumopt.kinoptpy.utils import norm, display_motion


class Contact:

    def __init__(self, position, start_time, end_time):
        self.pos = position
        self.init_time = start_time
        self.final_time = end_time

    def position(self):
        return self.pos

    def start_time(self):
        return self.init_time

    def end_time(self):
        return self.final_time


def get_contact_plan(contact_states, effs):
    contacts = {}
    for i, eff in enumerate(effs):
        num_contacts = len(contact_states(i))
        contacts[eff] = []
        for j in range(num_contacts):
            contact_ = contact_states(i)[j]
            start_time = contact_.start_time
            end_time = contact_.end_time
            position = contact_.position
            contacts[eff].append(Contact(position, start_time, end_time))

    return contacts


def Rquat(x,y,z,w):
    q = se3.Quaternion(w=w,x=x,y=y,z=z)
    q.normalize()
    return q.matrix()


def set_new_goal(goal_position):
    new_pos = np.zeros((3, 1))
    for i in range(3):
        new_pos[i, 0] = goal_position[i]

    return new_pos


def create_time_vector(dynamics_sequence):
    num_time_steps = len(dynamics_sequence.dynamics_states)
    # Create time vector
    time = np.zeros((num_time_steps))
    time[0] = dynamics_sequence.dynamics_states[0].dt
    for i in range(num_time_steps - 1):
        time[i + 1] = time[i] + dynamics_sequence.dynamics_states[i].dt

    return time


class KinematicsOptimizer:

    def __init__(self):
        self.kinematic_interface = None
        self.robot = None

    def initialize(self, planner_setting, max_iterations=1000, eps=0.01):
        # self.kinematic_interface = kin_interface
        self.num_time_steps = planner_setting.get(PlannerIntParam_NumTimesteps)

        # Load the robot from URDF-model
        urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/urdf/quadruped.urdf')
        self.robot = QuadrupedWrapper(urdf)
        self.max_iterations = max_iterations
        self.eps = eps

        self.in_contact = False
        # TODO: Set these using a config file
        self.offset = 0.0  # set constraint for z directly on floor
        self.q_max_delta = 10.0  # maximum difference in joint velocity
        self.dt = 0.01  # dt for the incremental steps for the inverse kinematics
        self.weights_value = 0.02  # value of the weights for all the tasks
        self.lambda_value = 0.0001  # value of the regularization

        self.kinematics_sequence = KinematicsSequence()
        self.kinematics_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                                        planner_setting.get(PlannerIntParam_NumDofs))

    def create_tasks(self, t, com_motion, contacts, eff_traj_poly):
        desired_velocities = []
        jacobians = []

        self.robot.transformations_dict["COM_GOAL"] = set_new_goal(com_motion[t, :])
        desired_vel_com = self.robot.get_desired_velocity(self.robot.transformations_dict["COM_GOAL"],
                                                          self.robot.transformations_dict["COM"], "TRANSLATION")

        for eff in self.robot.effs:
            self.in_contact = False
            phase = 0
            cnt_ = contacts[eff]

            for i in range(len(cnt_)):
                if cnt_[i].start_time() <= self.time[t] < cnt_[i].end_time():
                    phase = i
                    self.in_contact = True

            if self.in_contact:
                # Set goal position to contact position
                new_goal = set_new_goal(contacts[eff][phase].position())
            else:
                # Set goal position to minimum jerk trajectory of endeffector
                new_goal = [eff_traj_poly[eff][coord].eval(self.time[t]) for coord in range(3)]
                new_goal = set_new_goal(new_goal)

            self.robot.transformations_dict[eff + "_END_GOAL"] = new_goal
            desired_velocity = self.robot.get_desired_velocity(self.robot.transformations_dict[eff + "_END_GOAL"],
                                                               self.robot.transformations_dict[eff + "_END"], "TRANSLATION")
            desired_velocities.append(desired_velocity)
            jacobians.append(self.robot.jacobians_dict[eff + "_END"])

        desired_velocities.append(desired_vel_com)
        jacobians.append(self.robot.jacobians_dict["COM"])

        return desired_velocities, jacobians

    def create_constraints(self, z_floor, dt):
        # Create constraints for the robot end effectors, knees and hips to stay above the ground
        num_constraints = len(self.robot.effs) * len(self.robot.joints_list)
        G = np.zeros((num_constraints, self.robot.nv))
        z = np.zeros((num_constraints))
        index = 0
        for eff in self.robot.effs:
            for joint in self.robot.joints_list:
                joint_identifier = eff + "_" + joint
                G[index, :] = np.array(self.robot.jacobians_dict[joint_identifier]()[-1, :])[0]
                z[index] = self.robot.transformations_dict[joint_identifier]()[-1][0, 0]
                index += 1
        G *= - dt
        z_floor += self.offset
        h = - z_floor + z

        # Create constraints for limiting the joint velocity
        # G_max = np.eye(self.robot.nv)[- self.robot.num_ctrl_joints:, :]
        # G_min = - np.eye(self.robot.nv)[- self.robot.num_ctrl_joints:, :]
        # G = np.vstack((G, G_max, G_min))
        # h_max = self.q_max_delta * np.ones((self.robot.num_ctrl_joints, 1))
        # h_min = self.q_max_delta * np.ones((self.robot.num_ctrl_joints, 1))
        # h = np.vstack((np.resize(h, (h.shape[0], 1)), h_max, h_min))

        # h = np.squeeze(h)

        return G, h

    def optimize(self, ini_state, contact_sequence, dynamics_sequence):
        # Set floor's z coordinate to where the endeffector's z coordinates are
        z_floor = ini_state.eff(0)[-1]
        self.robot.viewer.gui.applyConfiguration('world/floor',[0.0, 0.0, z_floor,  0.0, 0.0, 0.0, 1.0])
        self.robot.viewer.gui.refresh()

        self.time = create_time_vector(dynamics_sequence)

        # Get COM motion
        com_motion = np.zeros((self.num_time_steps, 3))
        lmom = np.zeros((self.num_time_steps, 3))
        amom = np.zeros((self.num_time_steps, 3))
        for t in range(self.num_time_steps):
            com_motion[t, :] = dynamics_sequence.dynamics_states[t].com
            lmom[t, :] = dynamics_sequence.dynamics_states[t].lmom
            amom[t, :] = dynamics_sequence.dynamics_states[t].amom

        # import matplotlib.pyplot as plt

        # coordinates = ["y", "z"]

        # fig, axes = plt.subplots(2, 1, sharex='col')

        # for i, ax in enumerate(axes):
        #     coord_index = i + 1
        #     ax.plot(self.time, com_motion[:, coord_index], "b", label="COM_DYN_OPT")
        #     if coordinates[i] == "z":
        #         ax.plot([self.time[0], self.time[-1]], [z_floor, z_floor], "k", label="Floor")
        #     ax.legend()
        #     ax.set_ylabel(coordinates[i] + " [m]")

        # axes[-1].set_xlabel("t [s]")
        # fig.suptitle("COM")

        # plt.show()

        # Get contact plan
        contacts = get_contact_plan(contact_sequence.contact_states, self.robot.effs)

        # Generate minimum jerk trajectories
        z_max = max(com_motion[:, 2])
        z_min = min(com_motion[:, 2])
        eff_traj_poly = generate_eff_traj(contacts, z_max, z_min)

        # Create robot motion using IK for COM and endeffector trajectories
        ik = InverseKinematics(self.dt, self.robot.nq)
        q_traj = []

        num_uncontrolled_joints = self.robot.q.shape[0] - self.robot.num_ctrl_joints

        # Initialize dictionary for motion of all joints and COM
        self.ik_motion = {}
        self.ik_motion["COM"] = np.zeros((len(self.time), 3))
        self.ik_motion["LMOM"] = np.zeros((len(self.time), 3))
        self.ik_motion["AMOM"] = np.zeros((len(self.time), 3))
        self.ik_motion["base_position"] = np.zeros((len(self.time), 3))
        self.ik_motion["base_orientation"] = np.zeros((len(self.time), 4))
        self.ik_motion["joint_configurations"] = np.zeros((len(self.time), self.robot.num_ctrl_joints))
        self.ik_motion["base_linear_velocity"] = np.zeros((len(self.time), 3))
        self.ik_motion["base_angular_velocity"] = np.zeros((len(self.time), 3))
        self.ik_motion["joint_velocities"] = np.zeros((len(self.time), self.robot.num_ctrl_joints))
        for eff in self.robot.effs:
            for joint in self.robot.joints_list:
                joint_identifier = eff + "_" + joint
                self.ik_motion[joint_identifier] = np.zeros((len(self.time), 3))

        # q_dot = np.zeros((self.robot.model.nv))
        # q_before = self.robot.q.copy()

        for t in range(len(self.time)):
            i = 0
            print("time =", self.time[t])
            desired_velocities, jacobians = self.create_tasks(t, com_motion, contacts, eff_traj_poly)

            # Set weightsfor tasks
            weights = np.ones((len(jacobians)))
            if self.in_contact:
                weights[:] = self.weights_value
            else:
                weights[:] = self.weights_value
            weights[-1] = self.weights_value

            # Set regularizer
            lambda_ = self.lambda_value * np.ones_like(ik.lambda_)
            ik.set_regularizer(lambda_)

            # Add tasks to the cost function of the IK
            ik.add_tasks(desired_velocities, jacobians, gains=0.5, weights=weights)

            # q_before = self.robot.q.copy()
            # x_before, y_before, z_before, w_before = np.squeeze(np.array(q_before[3:7]), 1)
            # pdb.set_trace()
            # R_before = self.Rquat(x=x_before, y=y_before, z=z_before, w=w_before)

            # pdb.set_trace()

            q_previous = self.robot.q.copy()

            while norm(desired_velocities, ik.weights) > self.eps and i < self.max_iterations:
                if i % self.max_iterations == 0 and i > 0:
                    print("Error:", norm(desired_velocities))
                    print("Used iterations:", i)
                # x_current, y_current, z_current, w_current = np.squeeze(np.array(self.robot.q.copy()[3:7]), 1)
                # delta_q_ang_vel = se3.log3(self.Rquat(x=x_before, y=y_before, z=z_before, w=w_before).transpose() * R_before)
                G, h = self.create_constraints(z_floor, ik.dt)
                q_sol = ik.multi_task_IK(G=G, h=h, soft_constrained=True)
                # q_dot += q_sol
                self.robot.update_configuration(np.transpose(np.matrix(q_sol)) * ik.dt)
                i += 1
                # print(i)

            q_new = self.robot.q.copy()

            # q_after = self.robot.q.copy()
            # self.robot.set_configuration(q_before)
            # self.robot.update_configuration(np.transpose(np.matrix(q_dot)) * ik.dt)
            # # print(q_after)
            # print(q_after - self.robot.q)

            # self.robot.set_configuration(q_after)

            if t == 0:
                q_1 = self.robot.q
                q_dot = self.robot.get_difference(q_1, self.robot.q)
            else:
                q_1 = q_traj[-1]
                q_dot = self.robot.get_difference(q_1, self.robot.q) / (self.time[t] - self.time[t - 1])

            self.robot.centroidalMomentum(q_new, q_dot)

            self.ik_motion["COM"][t, :] = np.squeeze(self.robot.transformations_dict["COM"](), 1)
            self.ik_motion["LMOM"][t, :] = np.squeeze(self.robot.data.hg.vector[:3], 1)
            self.ik_motion["AMOM"][t, :] = np.squeeze(self.robot.data.hg.vector[3:], 1)

            self.ik_motion["base_position"][t, :] = np.squeeze(np.array(self.robot.q[:3]), 1)
            self.ik_motion["base_orientation"][t, :] = np.squeeze(np.array(self.robot.q[3:7]), 1)
            self.ik_motion["joint_configurations"][t, :] = np.squeeze(np.array(self.robot.q[num_uncontrolled_joints:]), 1)

            self.ik_motion["base_linear_velocity"][t, :] = np.squeeze(np.array(q_dot[:3]), 1)
            self.ik_motion["base_angular_velocity"][t, :] = np.squeeze(np.array(q_dot[3:6]), 1)
            self.ik_motion["joint_velocities"][t, :] = np.squeeze(np.array(q_dot[num_uncontrolled_joints - 1:]), 1)
            for eff in self.robot.effs:
                for joint in self.robot.joints_list:
                    joint_identifier = eff + "_" + joint
                    self.ik_motion[joint_identifier][t, :] = np.squeeze(self.robot.transformations_dict[joint_identifier](), 1)

            print("Finished after iteration:", i)
            q_traj.append(self.robot.q)
            ik.delete_tasks()
            self.robot.display(self.robot.q)

        # display_motion(self.robot, q_traj, self.time)

        q_matrix = np.zeros((len(q_traj), q_traj[0].shape[0]))
        for i in range(len(q_traj)):
            q_matrix[i, :] = np.squeeze(np.array(q_traj[i]))

        self.populate_sequence()

        # self.plot_plan(com_motion, lmom, amom, eff_traj_poly, z_floor)

    def populate_sequence(self):
        for time_id in range(self.num_time_steps):
            kinematic_state = self.kinematics_sequence.kinematics_states[time_id]
            kinematic_state.com = self.ik_motion["COM"][time_id, :]
            kinematic_state.lmom = self.ik_motion["LMOM"][time_id, :]
            kinematic_state.amom = self.ik_motion["AMOM"][time_id, :]

            kinematic_state.robot_posture.base_position = self.ik_motion["base_position"][time_id, :]
            kinematic_state.robot_posture.base_orientation = self.ik_motion["base_orientation"][time_id, :]
            kinematic_state.robot_posture.joint_positions = self.ik_motion["joint_configurations"][time_id, :]

            kinematic_state.robot_velocity.base_linear_velocity = self.ik_motion["base_linear_velocity"][time_id, :]
            kinematic_state.robot_velocity.base_angular_velocity = self.ik_motion["base_angular_velocity"][time_id, :]
            kinematic_state.robot_velocity.joint_velocities = self.ik_motion["joint_velocities"][time_id, :]


    def plot_plan(self, com_motion, lmom, amom, eff_traj_poly, z_floor):
        import matplotlib.pyplot as plt

        coordinates = ["y", "z"]

        fig, axes = plt.subplots(2, 1, sharex='col')

        for i, ax in enumerate(axes):
            coord_index = i + 1
            ax.plot(self.time, self.ik_motion["COM"][:, coord_index], "r", label="COM_IK")
            ax.plot(self.time, com_motion[:, coord_index], "b", label="COM_DYN_OPT")
            if coordinates[i] == "z":
                ax.plot([self.time[0], self.time[-1]], [z_floor, z_floor], "k", label="Floor")
            ax.legend()
            ax.set_ylabel(coordinates[i] + " [m]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("COM")

        fig, axes = plt.subplots(2, 1, sharex='col')

        for i, ax in enumerate(axes):
            coord_index = i + 1
            ax.plot(self.time, self.ik_motion["LMOM"][:, coord_index], "r", label="LMOM_IK")
            ax.plot(self.time, lmom[:, coord_index], "b", label="LMOM_DYN_OPT")
            ax.legend()
            ax.set_ylabel(coordinates[i] + " [m]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("LMOM")

        fig, axes = plt.subplots(2, 1, sharex='col')

        for i, ax in enumerate(axes):
            coord_index = i + 1
            ax.plot(self.time, self.ik_motion["AMOM"][:, coord_index], "r", label="AMOM_IK")
            ax.plot(self.time, amom[:, coord_index], "b", label="AMOM_DYN_OPT")
            ax.legend()
            ax.set_ylabel(coordinates[i] + " [m]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("AMOM")

        # TODO: add planned minimum jerk trajectories for end effectors
        for joint in self.robot.joints_list:
            fig, axes = plt.subplots(2, 1, sharex='col')
            for eff in self.robot.effs:
                joint_identifier = eff + "_" + joint
                for i, ax in enumerate(axes):
                    coord_index = i + 1
                    ax.plot(self.time, self.ik_motion[joint_identifier][:, coord_index], label=joint_identifier+"_IK")
                    ax.set_ylabel(coordinates[i] + " [m]")

            axes[-1].plot([self.time[0], self.time[-1]], [z_floor, z_floor], "k", label="Floor")
            for ax in axes:
                ax.legend()

            axes[-1].set_xlabel("t [s]")
            fig.suptitle(joint)

        plt.show()
