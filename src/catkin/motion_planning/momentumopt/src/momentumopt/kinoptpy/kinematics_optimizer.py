import os
import numpy as np

import pinocchio as se3
from pinocchio.utils import *
from pymomentum import *

from momentumopt.kinoptpy.min_jerk_traj import generate_eff_traj
from quadruped.quadruped_wrapper import QuadrupedWrapper
from momentumopt.kinoptpy.inv_kin import InverseKinematics
from momentumopt.kinoptpy.utils import norm, display_motion, norm_momentum


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

    def initialize(self, planner_setting, max_iterations=50, eps=0.001):
        # self.kinematic_interface = kin_interface
        self.num_time_steps = planner_setting.get(PlannerIntParam_NumTimesteps)
        self.robot_weight = planner_setting.get(PlannerDoubleParam_RobotWeight)

        # Load the robot from URDF-model
        urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/urdf/quadruped.urdf')
        # print("urdf_path:", urdf)

        self.robot = QuadrupedWrapper(urdf)
        #TODO: naming convention must be improved
        self.max_iterations = max_iterations
        self.eps = eps

        self.in_contact = [False] * len(self.robot.effs)
        # TODO: Set these using a config file
        self.offset = 0.0  # set constraint for z directly on floor
        self.q_max_delta = 10.0  # maximum difference in joint velocity
        self.dt = planner_setting.get(PlannerDoubleParam_TimeStep)  # dt for the incremental steps for the inverse kinematics
        self.weights_value = 10.0  # value of the weights for all the tasks
        self.lambda_value = 0.0001  # value of the regularization
        self.w_not_in_contact = 1.0  # weight for non active end effector tracking

        self.kinematics_sequence = KinematicsSequence()
        self.kinematics_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                                        planner_setting.get(PlannerIntParam_NumDofs))

    def create_tasks(self, t, com_motion, contacts, eff_traj_poly):
        desired_positions = []
        desired_velocities = []
        jacobians = []

        self.robot.transformations_dict["COM_GOAL"] = set_new_goal(com_motion[t, :])
        desired_vel_com = self.robot.get_desired_velocity(self.robot.transformations_dict["COM_GOAL"],
                                                          self.robot.transformations_dict["COM"], "TRANSLATION")

        for eff_id, eff in enumerate(self.robot.effs ):
            self.in_contact[eff_id] = False
            phase = 0
            cnt_ = contacts[eff]

            for i in range(len(cnt_)):
                if cnt_[i].start_time() <= self.time[t] < cnt_[i].end_time():
                    phase = i
                    self.in_contact[eff_id] = True

            if self.in_contact[eff_id]:
                # Set goal position to contact position
                new_goal = set_new_goal(contacts[eff][phase].position())
            else:
                # Set goal position to minimum jerk trajectory of endeffector
                new_goal = [eff_traj_poly[eff][coord].eval(self.time[t]) for coord in range(3)]
                new_goal = set_new_goal(new_goal)


            desired_positions.append(np.reshape((self.robot.transformations_dict[eff + "_ANKLE"]()),(1,3)))

            self.robot.transformations_dict[eff + "_END_GOAL"] = new_goal
            desired_velocity = self.robot.get_desired_velocity(self.robot.transformations_dict[eff + "_END_GOAL"],
                                                               self.robot.transformations_dict[eff + "_ANKLE"], "TRANSLATION")
            desired_velocities.append(desired_velocity)
            jacobians.append(self.robot.jacobians_dict[eff + "_ANKLE"])

        desired_positions = np.squeeze(desired_positions, axis=0)
        desired_positions = np.reshape(desired_positions, (1,12))
        desired_velocities.append(desired_vel_com)
        jacobians.append(self.robot.jacobians_dict["COM"])

        return desired_positions, desired_velocities, jacobians


    def create_constraints(self, z_floor, dt, acceleration=False):
        # Create constraints for the robot end effectors, knees and hips to stay above the ground
        num_constraints = len(self.robot.effs ) * len(self.robot.joints_list)
        G = np.zeros((num_constraints, self.robot.robot.nv))
        z = np.zeros((num_constraints))
        index = 0
        for eff in self.robot.effs :
            for joint in self.robot.joints_list:
                joint_identifier = eff + "_" + joint
                G[index, :] = np.array(self.robot.jacobians_dict[joint_identifier]()[-1, :])[0]
                z[index] = self.robot.transformations_dict[joint_identifier]()[-1][0, 0]
                index += 1
        G *= - dt
        z_floor += self.offset
        h = - z_floor + z

        if acceleration:
            # add constraints for connection of acceleration and velocity
            G = np.hstack((G, np.zeros_like(G)))
            b = np.squeeze(np.array(self.robot.dq), 1)
            A = np.hstack((np.eye(self.robot.robot.nv), - dt * np.eye(self.robot.robot.nv)))
        else:
            A = None
            b = None

        # Create constraints for limiting the joint velocity
        # G_max = np.eye(self.robot.robot.nv)[- self.robot.num_ctrl_joints:, :]
        # G_min = - np.eye(self.robot.robot.nv)[- self.robot.num_ctrl_joints:, :]
        # G = np.vstack((G, G_max, G_min))
        # h_max = self.q_max_delta * np.ones((self.robot.num_ctrl_joints, 1))
        # h_min = self.q_max_delta * np.ones((self.robot.num_ctrl_joints, 1))
        # h = np.vstack((np.resize(h, (h.shape[0], 1)), h_max, h_min))

        # h = np.squeeze(h)

        return G, h, A, b

    def optimize(self, ini_state, contact_sequence, dynamics_sequence, plotting=False):

        # Set floor's z     coordinate to where the endeffector's z coordinates are
        z_floor = ini_state.eff(0)[-1]
        self.robot.robot.viewer.gui.applyConfiguration('world/floor',[0.0, 0.0, z_floor,  0.0, 0.0, 0.0, 1.0])
        self.robot.robot.viewer.gui.refresh()

        self.time = create_time_vector(dynamics_sequence)

        # Get COM motion
        com_motion = np.zeros((self.num_time_steps, 3))
        lmom = np.zeros((self.num_time_steps, 3))
        amom = np.zeros((self.num_time_steps, 3))
        for t in range(self.num_time_steps):
            com_motion[t, :] = dynamics_sequence.dynamics_states[t].com
            lmom[t, :] = dynamics_sequence.dynamics_states[t].lmom
            amom[t, :] = dynamics_sequence.dynamics_states[t].amom

        planned_forces = np.zeros((self.num_time_steps, 3 * len(self.robot.effs)))
        for time_id in range(self.num_time_steps):
            for eff_id, eff in enumerate(self.robot.effs):
                eff = eff + "_ANKLE"
                force = dynamics_sequence.dynamics_states[time_id].effForce(eff_id) * self.robot_weight
                planned_forces[time_id, eff_id * 3 : eff_id * 3 + 3] = force

        import matplotlib.pyplot as plt

        coordinates = ["y", "z"]

        fig, axes = plt.subplots(2, 1, sharex='col')

        for i, ax in enumerate(axes):
            coord_index = i + 1
            ax.plot(self.time, com_motion[:, coord_index], "b", label="COM_DYN_OPT")
            if coordinates[i] == "z":
                ax.plot([self.time[0], self.time[-1]], [z_floor, z_floor], "k", label="Floor")
            ax.legend()
            ax.set_ylabel(coordinates[i] + " [m]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("COM")

        plt.show()

        # Get contact plan
        contacts = self.contacts = get_contact_plan(contact_sequence.contact_states, self.robot.effs)

        # Generate minimum jerk trajectories
        z_max = max(com_motion[:, 2])
        z_min = min(com_motion[:, 2])
        self.eff_traj_poly = eff_traj_poly = generate_eff_traj(contacts, z_max, z_min)

        # Create robot motion using IK for COM and endeffector trajectories
        ik = InverseKinematics(self.dt, self.robot.robot.nq)
        q_traj = []
        q_vel = []

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
        self.ik_motion["base_linear_acceleration"] = np.zeros((len(self.time), 3))
        self.ik_motion["base_angular_acceleration"] = np.zeros((len(self.time), 3))
        self.ik_motion["joint_accelerations"] = np.zeros((len(self.time), self.robot.num_ctrl_joints))
        for eff in self.robot.effs :
            for joint in self.robot.joints_list:
                joint_identifier = eff + "_" + joint
                self.ik_motion[joint_identifier] = np.zeros((len(self.time), 3))

        ## adding variables to store end_eff trajectories and COM (position and Vel)
        ## For impledance control

        self.motion_eff = {}
        #self.effs = ["BR", "BL", "FR", "FL"]  # order is important
        self.motion_eff["trajectory"] = np.zeros((len(self.time), len(self.robot.effs) * 3))
        self.motion_eff["velocity"] = np.zeros((len(self.time), len(self.robot.effs) * 3))
        self.motion_eff["COM"] = np.zeros((len(self.time), 7)) # xyzq
        self.motion_eff["COM_vel"] = np.zeros((len(self.time), 3))
        ## This takes end effectoer position wrt to base of foot (take a look at leg impedance)
        self.motion_eff["trajectory_wrt_base"] = np.zeros((len(self.time), len(self.robot.effs) * 3))
        self.motion_eff["velocity_wrt_base"] = np.zeros((len(self.time), len(self.robot.effs) * 3))

        def get_effs_vel(desired_velocities, t):
            ## retruns the value of the end effectors velocity at time t
            vel = np.concatenate((np.concatenate((desired_velocities[0](t), desired_velocities[1](t))),
                                np.concatenate((desired_velocities[2](t), desired_velocities[3](t)))
            ))

            return np.reshape(vel, (1, len(self.robot.effs)*3))


        def get_desired_com_vel(desired_vel, t):
            return np.reshape(desired_vel[4](t), (1,3))


        # set initial robot configuration
        t = 0

        #self.motion_eff["trajectory"][t] = get_effs_traj(eff_traj_poly, t)


        desired_positions, desired_velocities, jacobians = self.create_tasks(t, com_motion, contacts, eff_traj_poly)

        self.motion_eff["trajectory"][t] = desired_positions
        self.motion_eff["velocity"][t] = get_effs_vel(desired_velocities, self.dt)
        self.motion_eff["COM_vel"][t] = get_desired_com_vel(desired_velocities, self.dt)
        #self.motion_eff["COM"][t] = np.hstack((com_motion[t], np.squeeze(np.array(self.robot.q[3:7]), 1)))
        self.motion_eff["COM"][t] = np.squeeze(self.robot.q[0:7], 1)
        # Set regularizer
        lambda_ = self.lambda_value * np.ones_like(ik.lambda_)
        ik.set_regularizer(lambda_)

        weights = self.weights_value * np.ones((len(jacobians)))
        # Add tasks to the cost function of the IK
        ik.add_tasks(desired_velocities, jacobians,
                     gains=0.5, weights=weights)

        q_dot = self.robot.get_difference(self.robot.q, self.robot.q)
        self.robot.set_velocity(q_dot)

        i = 0
        while norm(desired_velocities, ik.weights) > self.eps and i < self.max_iterations:
            if i % self.max_iterations == 0 and i > 0:
                print("Error:", norm(desired_velocities))
                print("Used iterations:", i)
            G, h, A, b = self.create_constraints(z_floor, ik.dt)
            ## q_sol = dq
            q_sol = ik.multi_task_IK(G=G, h=h, soft_constrained=True)
            self.robot.update_configuration(np.transpose(np.matrix(q_sol)) * ik.dt)
            i += 1



        # desired_delta_momentum_array = np.zeros((len(self.time), 6))
        # actual_delta_momentum_array = np.zeros((len(self.time), 6))

        # lambda_ = self.lambda_value * np.ones(self.robot.robot.nv * 2) # np.ones_like(ik.lambda_)
        # ik.set_regularizer(lambda_)

        # Track robot motion
        for t in range(len(self.time)):
            i = 0
            #print("time =", self.time[t])

            desired_positions, desired_velocities, jacobians = self.create_tasks(t, com_motion, contacts, eff_traj_poly)
            self.motion_eff["trajectory"][t] = desired_positions
            self.motion_eff["velocity"][t] = get_effs_vel(desired_velocities, self.dt)
            self.motion_eff["COM_vel"][t] = get_desired_com_vel(desired_velocities, self.dt)
#            self.motion_eff["COM"][t] = np.hstack((com_motion[t], np.squeeze(np.array(self.robot.q[3:7]), 1)))
            self.motion_eff["COM"][t] = np.squeeze(self.robot.q[0:7], 1)


            #print(t, self.motion_eff["trajectory"][t])

            if t == 0:
                delta_momentum = np.zeros((6))
            else:
                delta_momentum = np.hstack((lmom[t, :], amom[t, :]))

            # Set weightsfor tasks
            weights = 0.01 * self.weights_value * np.ones((len(jacobians)))

            for i in range(len(self.robot.effs )):
                if self.in_contact[i]:
                    weights[i] = 0.01 * self.weights_value
                else:
                    weights[i] = 0.01 * self.w_not_in_contact * self.weights_value
                #weights[-1] = self.weights_value

            # Set regularizer
            lambda_ = self.lambda_value * np.ones_like(ik.lambda_)
            ik.set_regularizer(lambda_)

            # Add tasks to the cost function of the IK
            # ik.add_tasks(desired_velocities, jacobians, gains=0.5, weights=weights)
            ik.add_tasks(desired_velocities, jacobians,
                         # centroidal_momentum=self.robot.centroidal_momentum,
                         # d_centroidal_momentum=self.robot.d_centroidal_momentum,
                         # desired_momentum=delta_momentum,
                         gains=0.5, weights=weights)

            # if t == 0:
            #     ik.centroidal_momentum = None

            q_previous = self.robot.q.copy()

            if t <= 0:
                q_dot = self.robot.get_difference(self.robot.q, self.robot.q)
                q_dd = self.robot.get_difference(self.robot.dq, self.robot.dq)
                self.robot.set_velocity(q_dot)
                self.robot.set_acceleration(q_dd)

            # while norm_momentum(np.dot(np.array(self.robot.centroidal_momentum()), np.squeeze(np.array(self.robot.dq), 1)), desired_momentum) > self.eps and i < 100:
            # while norm(desired_velocities, ik.weights) + norm_momentum(np.dot(np.array(self.robot.centroidal_momentum()), np.squeeze(np.array(self.robot.dq), 1)), desired_momentum) > self.eps and i < 50:
            while norm(desired_velocities, ik.weights) > self.eps and i < self.max_iterations:
                if i % self.max_iterations == 0 and i > 0:
                    print("Error:", norm(desired_velocities))
                    print("Used iterations:", i)
                if t == 0:
                    delta_momentum = np.zeros((6))
                else:
                    q_dot_intermediate = self.robot.get_difference(q_traj[-1], self.robot.q) / (self.time[t - 1] + ik.dt * max(i, 1) - self.time[t - 1])
                    self.robot.robot.centroidalMomentum(self.robot.q, q_dot_intermediate)
                    delta_momentum = np.hstack((lmom[t, :], amom[t, :])) - np.squeeze(np.array(self.robot.data.hg.vector), 1)
                ik.desired_momentum = delta_momentum
                # delta_momentum = desired_momentum - np.dot(np.array(self.robot.centroidal_momentum()), np.squeeze(np.array(self.robot.dq), 1))
                # print(delta_momentum[1:4])
                # ik.desired_momentums = delta_momentum
                G, h, A, b = self.create_constraints(z_floor, ik.dt)
                q_sol = ik.multi_task_IK(G=G, h=h, soft_constrained=True)
                # q_sol = ik.multi_task_IK_momentum(G=G, h=h, A=A, b=b, soft_constrained=True)
                # q_sol_vel = q_sol[:14]
                self.robot.update_configuration(np.transpose(np.matrix(q_sol)) * ik.dt)
                # if t == 0:
                #     delta_t = (self.time[t] - 0.0)
                # else:
                #     delta_t = (self.time[t] - self.time[t - 1])
                # q_diff = self.robot.get_difference(self.robot.q, q_previous) / ((i + 1) * ik.dt)
                # self.robot.set_velocity(q_diff)

                i += 1

                #print("1", self.robot.com(q))
                #print("2", self.robot.q)

            # actual_delta_momentum_array

            q_new = self.robot.q.copy()

            if t > 0:
                q_dot = self.robot.get_difference(q_traj[-1], self.robot.q) / (self.time[t] - self.time[t - 1])
                q_dd = (q_dot - q_vel[-1]) / (self.time[t] - self.time[t - 1])
                self.robot.set_velocity(q_dot)
                self.robot.set_acceleration(q_dd)

            self.robot.robot.centroidalMomentum(q_new, q_dot)

            self.ik_motion["COM"][t, :] = np.squeeze(self.robot.robot.com(q_new), 1)
            self.ik_motion["LMOM"][t, :] = np.squeeze(self.robot.data.hg.vector[:3], 1)
            self.ik_motion["AMOM"][t, :] = np.squeeze(self.robot.data.hg.vector[3:], 1)

            self.ik_motion["base_position"][t, :] = np.squeeze(np.array(self.robot.q[:3]), 1)
            self.ik_motion["base_orientation"][t, :] = np.squeeze(np.array(self.robot.q[3:7]), 1)
            self.ik_motion["joint_configurations"][t, :] = np.squeeze(np.array(self.robot.q[num_uncontrolled_joints:]), 1)

            self.ik_motion["base_linear_velocity"][t, :] = np.squeeze(np.array(q_dot[:3]), 1)
            self.ik_motion["base_angular_velocity"][t, :] = np.squeeze(np.array(q_dot[3:6]), 1)
            self.ik_motion["joint_velocities"][t, :] = np.squeeze(np.array(q_dot[num_uncontrolled_joints - 1:]), 1)

            self.ik_motion["base_linear_acceleration"][t, :] = np.squeeze(np.array(q_dd[:3]), 1)
            self.ik_motion["base_angular_acceleration"][t, :] = np.squeeze(np.array(q_dd[3:6]), 1)
            self.ik_motion["joint_accelerations"][t, :] = np.squeeze(np.array(q_dd[num_uncontrolled_joints - 1:]), 1)

            for eff in self.robot.effs :
                for joint in self.robot.joints_list:
                    joint_identifier = eff + "_" + joint
                    self.ik_motion[joint_identifier][t, :] = np.squeeze(self.robot.transformations_dict[joint_identifier](), 1)

            # print("Finished after iteration:", i)
            q_traj.append(self.robot.q.copy())
            q_vel.append(self.robot.dq.copy())
            ik.delete_tasks()
            self.robot.display(self.robot.q)



        # ### Converting position and velocities relative to the base

        pos_hips_base = np.matrix([  [0.1, -0.1, 0.1, -0.1],
                                    [-0.2, -0.2, 0.2, 0.2],
                                    [0.0, 0.0, 0.0, 0.0]])

        pos_hips_world_arr_BL = []
        pos_hips_world_arr_FL = []
        for t in range(0,len(self.time)):
            #xyzq = np.reshape(self.motion_eff["COM"][t], (7,1))
            xyzq = np.reshape(self.motion_eff["COM"][t], (7,1))

            homo = XYZQUATToSe3(xyzq)
            ## position of hip(BL, BR, FL, FR) wrt world
            pos_hips_world = np.add(np.matmul(homo.rotation, pos_hips_base), np.transpose([com_motion[t]]))
            pos_hips_world_arr_BL.append(np.transpose(pos_hips_world[: ,0]))
            pos_hips_world_arr_FL.append(np.transpose(pos_hips_world[: ,2]))
            self.motion_eff["trajectory_wrt_base"][t][0:3] = np.subtract(self.motion_eff["trajectory"][t][0:3], np.transpose(pos_hips_world[: ,0]))
            self.motion_eff["trajectory_wrt_base"][t][3:6] = np.subtract(self.motion_eff["trajectory"][t][3:6], np.transpose(pos_hips_world[: ,1]))
            self.motion_eff["trajectory_wrt_base"][t][6:9] = np.subtract(self.motion_eff["trajectory"][t][6:9], np.transpose(pos_hips_world[: ,2]))
            self.motion_eff["trajectory_wrt_base"][t][9:12] = np.subtract(self.motion_eff["trajectory"][t][9:12], np.transpose(pos_hips_world[: ,3]))



        self.motion_eff["trajectory_wrt_base"][: ,[0,1,2]] = np.subtract(np.subtract(self.motion_eff["trajectory"][: ,[0,1,2]], [0.1, -0.2, 0.0]),com_motion)
        self.motion_eff["trajectory_wrt_base"][: ,[3,4,5]] = np.subtract(np.subtract(self.motion_eff["trajectory"][: ,[3,4,5]], [-0.1, -.2, 0]),com_motion)
        self.motion_eff["trajectory_wrt_base"][: ,[6,7,8]] = np.subtract(np.subtract(self.motion_eff["trajectory"][: ,[6,7,8]], [0.1, .2, 0]),com_motion)
        self.motion_eff["trajectory_wrt_base"][: ,[9,10,11]] = np.subtract(np.subtract(self.motion_eff["trajectory"][: ,[9,10,11]], [-0.1, .2, 0]),com_motion)

        ## Not sure about the 0.27 division. Mass is 2.7 but after plotting the ration between lmom and com_vel 0.27 is observed
        self.motion_eff["velocity_wrt_base"][: ,[0,1,2]] = np.subtract(self.motion_eff["velocity"][: ,[0,1,2]], np.divide(lmom, 0.22))
        self.motion_eff["velocity_wrt_base"][: ,[3,4,5]] = np.subtract(self.motion_eff["velocity"][: ,[3,4,5]], np.divide(lmom, 0.22))
        self.motion_eff["velocity_wrt_base"][: ,[6,7,8]] = np.subtract(self.motion_eff["velocity"][: ,[6,7,8]], np.divide(lmom, 0.22))
        self.motion_eff["velocity_wrt_base"][: ,[9,10,11]] = np.subtract(self.motion_eff["velocity"][: ,[9,10,11]], np.divide(lmom, 0.22))

        q_matrix = np.zeros((len(q_traj), q_traj[0].shape[0]))
        for i in range(len(q_traj)):
            q_matrix[i, :] = np.squeeze(np.array(q_traj[i]))

        self.q_matrix = q_matrix
        self.populate_sequence()

        if plotting:
            self.plot_com_eff(com_motion, pos_hips_world_arr_BL, pos_hips_world_arr_FL)
            self.plot_plan(com_motion, lmom, amom, eff_traj_poly, z_floor, contacts)
            self.plot_desired_forces(dynamics_sequence, contacts)

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

            kinematic_state.robot_acceleration.base_linear_acceleration = self.ik_motion["base_linear_acceleration"][time_id, :]
            kinematic_state.robot_acceleration.base_angular_acceleration = self.ik_motion["base_angular_acceleration"][time_id, :]
            kinematic_state.robot_acceleration.joint_accelerations = self.ik_motion["joint_accelerations"][time_id, :]


    def get_swing_times(self, contacts):
        swing_times = {}

        for eff in self.robot.effs :
            swing_times[eff] = []
            num_contacts = len(contacts[eff])
            if num_contacts > 1:
                for i in range(len(contacts[eff]) - 1):
                    swing_times[eff].append([contacts[eff][i].end_time(), contacts[eff][i + 1].start_time()])

        return swing_times

    def plot_com_eff(self, com_motion, pos_hips_world_arr_BL, pos_hips_world_arr_FL):
        import matplotlib.pyplot as plt
        # ## plots for des_position and velocity vas actual values. and Hip and Knee torques
        pos_hips_world_arr_BL = np.reshape(pos_hips_world_arr_BL, (len(pos_hips_world_arr_BL), 3))
        pos_hips_world_arr_FL = np.reshape(pos_hips_world_arr_FL, (len(pos_hips_world_arr_FL), 3))
        # print(np.shape(pos_hips_world_arr_BL))
        fig1, ax1 = plt.subplots(7,1, sharex = True)
        #
        ax1[0].plot(self.time, com_motion[: ,0] , color="black", label = "com_y")
        ax1[0].plot(self.time, com_motion[: ,1] , color="green", label = "com_x")
        ax1[0].plot(self.time, com_motion[: ,2] , color="red", label = "com_z")
        ax1[0].legend()
        ax1[0].set_xlabel("millisec")
        ax1[0].set_ylabel("m")
        ax1[0].grid()


        ax1[1].plot(self.time, pos_hips_world_arr_BL[: ,0], color = "black", label = "BLH_y")
        ax1[1].plot(self.time, pos_hips_world_arr_BL[: ,1], color = "green", label = "BLH_x")
        ax1[1].plot(self.time, pos_hips_world_arr_BL[: ,2], color = "red", label = "BLH_z")
        #ax1[0].plot(rel_pos_foot_z, color = "red", label = "actual_foot_pos_z")
        ax1[1].legend()
        ax1[1].set_xlabel("millisec")
        ax1[1].set_ylabel("m")
        ax1[1].grid()

        ax1[2].plot(self.time, self.motion_eff["trajectory"][ :, 0], color = "black", label = "BL_y")
        ax1[2].plot(self.time, self.motion_eff["trajectory"][ :, 1], color = "green", label = "BL_x")
        ax1[2].plot(self.time, self.motion_eff["trajectory"][ :, 2], color = "red", label = "BL_z")
        #ax1[0].plot(rel_pos_foot_z, color = "red", label = "actual_foot_pos_z")
        ax1[2].legend()
        ax1[2].set_xlabel("millisec")
        ax1[2].set_ylabel("m")
        ax1[2].grid()

        ax1[3].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 0] , color="black", label = "BL_wrt_base_y")
        ax1[3].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 1] , color="green", label = "BL_wrt_base_x")
        ax1[3].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 2] , color="red", label = "BL_wrt_base_z")
        ax1[3].legend()
        ax1[3].set_xlabel("millisec")
        ax1[3].set_ylabel("m")
        ax1[3].grid()

        ax1[4].plot(self.time, pos_hips_world_arr_FL[: ,0] , color="black", label = "FLH_y")
        ax1[4].plot(self.time, pos_hips_world_arr_FL[: ,1] , color="green", label = "FLH_x")
        ax1[4].plot(self.time, pos_hips_world_arr_FL[: ,2] , color="red", label = "FLH_z")
        ax1[4].legend()
        ax1[4].set_xlabel("millisec")
        ax1[4].set_ylabel("m")
        ax1[4].grid()


        ax1[5].plot(self.time, self.motion_eff["trajectory"][ :, 6], color = "black", label = "FL_y")
        ax1[5].plot(self.time, self.motion_eff["trajectory"][ :, 7], color = "green", label = "FL_x")
        ax1[5].plot(self.time, self.motion_eff["trajectory"][ :, 8], color = "red", label = "FL_z")
        #ax1[0].plot(rel_pos_foot_z, color = "red", label = "actual_foot_pos_z")
        ax1[5].legend()
        ax1[5].set_xlabel("millisec")
        ax1[5].set_ylabel("m")
        ax1[5].grid()

        ax1[6].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 6] , color="black", label = "FL_wrt_base_y")
        ax1[6].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 7] , color="green", label = "FL_wrt_base_x")
        ax1[6].plot(self.time, self.motion_eff["trajectory_wrt_base"][:, 8] , color="red", label = "FL_wrt_base_z")

        ax1[6].legend()
        ax1[6].set_xlabel("millisec")
        ax1[6].set_ylabel("rad/sec")
        ax1[6].grid()

        plt.show()

    def plot_plan(self, com_motion, lmom, amom, eff_traj_poly, z_floor, contacts):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        import matplotlib as mpl

        coordinates = ["x", "y", "z"]
        swing_times = self.get_swing_times(contacts)

        fig, axes = plt.subplots(len(coordinates), 1, sharex='col')

        for i, ax in enumerate(axes):
            ax.plot(self.time, self.ik_motion["COM"][:, i], "r", label="COM_IK")
            ax.plot(self.time, com_motion[:, i], "b", label="COM_DYN_OPT")

            for eff in self.robot.effs :
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            ax.legend()
            ax.set_ylabel(coordinates[i] + " [m]")

        axes[-1].axhline(y=z_floor, color="k", linestyle="-", label="Floor")
        axes[-1].set_xlabel("t [s]")
        fig.suptitle("COM")

        fig, axes = plt.subplots(len(coordinates), 1, sharex='col')

        for i, ax in enumerate(axes):
            ax.plot(self.time, self.ik_motion["LMOM"][:, i], "r", label="LMOM_IK")
            ax.plot(self.time, lmom[:, i], "b", label="LMOM_DYN_OPT")

            for eff in self.robot.effs :
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            ax.legend()
            ax.set_ylabel("p_" + coordinates[i] + " [kg * m / s]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("LMOM")

        fig, axes = plt.subplots(len(coordinates), 1, sharex='col')

        for i, ax in enumerate(axes):
            ax.plot(self.time, self.ik_motion["AMOM"][:, i], "r", label="AMOM_IK")
            ax.plot(self.time, amom[:, i], "b", label="AMOM_DYN_OPT")

            for eff in self.robot.effs :
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            ax.legend()
            ax.set_ylabel("L_" + coordinates[i] + " [kg * m^2 / s]")

        axes[-1].set_xlabel("t [s]")
        fig.suptitle("AMOM")

        fig, axes = plt.subplots(2, 1, sharex='col')

        # # Compute the desired endeffector trajectory.
        # self.eff_traj = np.zeros((self.robot.effs, self.num_time_steps, 3))
        # for eff_id, eff in enumerate(self.robot.effs):
        #   for t in range(self.num_time_steps):
        #     phase = 0
        #     self.in_contact[eff_id] = False
        #     cnt_ = contacts[eff]

        #     for i in range(len(cnt_)):
        #         if cnt_[i].start_time() <= self.time[t] < cnt_[i].end_time():
        #             phase = i
        #             self.in_contact[eff_id] = True

        #     if self.in_contact[eff_id]:
        #         # Set goal position to contact position
        #         self.eff_traj[eff_id, t] = set_new_goal(contacts[eff][phase].position())
        #     else:
        #         # Set goal position to minimum jerk trajectory of endeffector
        #         self.eff_traj[eff_id, t] = [eff_traj_poly[eff][coord].eval(self.time[t]) for coord in range(3)]

        #     axes[0].plot(self.eff_traj[eff_id, :, 1], label='eff_{}_y'.format(eff))
        #     axes[1].plot(self.eff_traj[eff_id, :, 2], label='eff_{}_z'.format(eff))

        # fig.suptitle("eff_plan")

        # TODO: add planned minimum jerk trajectories for end effectors
        for joint in self.robot.joints_list:
            fig, axes = plt.subplots(2, 1, sharex='col')
            for eff in self.robot.effs :
                joint_identifier = eff + "_" + joint
                for i, ax in enumerate(axes):
                    coord_index = i + 1
                    ax.plot(self.time, self.ik_motion[joint_identifier][:, coord_index], color=self.robot.colors[eff], label=joint_identifier+"_IK")
                    ax.set_ylabel(coordinates[i + 1] + " [m]")

            for eff in self.robot.effs :
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    axes[0].axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    axes[0].axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    axes[1].axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    axes[1].axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            axes[-1].axhline(y=z_floor, color="k", linestyle="-", label="Floor")

            for ax in axes:
                ax.legend()

            axes[-1].set_xlabel("t [s]")
            fig.suptitle(joint)

        plt.show()

    def plot_desired_forces(self, dynamics_sequence, contacts):
        import matplotlib.pyplot as plt
        num_effs = len(self.robot.effs )
        eff_forces = np.zeros((len(self.time), num_effs, 3))
        for eff_id in range(num_effs):
            for time_id in range(len(self.time)):
                eff_forces[time_id, eff_id, :] = dynamics_sequence.dynamics_states[time_id].effForce(eff_id) * self.robot_weight

        swing_times = self.get_swing_times(contacts)

        fig, axes = plt.subplots(2, 2, sharex='col')

        for i in range(num_effs):
            idx_1, idx_2 = np.unravel_index([i], (2, 2))
            ax = axes[idx_1[0], idx_2[0]]
            ax.plot(self.time, eff_forces[:, i, -1])

            joint_name = self.robot.effs [i] + "_ANKLE"
            ax.set_title(joint_name)

            for eff in self.robot.effs :
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            ax.set_ylabel("F_z [N]")

            if idx_1[0] == 1:
                ax.set_xlabel("t [s]")

        fig.suptitle("Desired Forces")

        plt.show()
