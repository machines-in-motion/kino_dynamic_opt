import os
import numpy as np
from time import sleep, time
import pdb

import pybullet as p

from src.quadruped.quadruped_wrapper import QuadrupedWrapper
# from src.momentumexe.simulator import Simulator


np.set_printoptions(precision=2, suppress=True)


class PDController(object):
    def __init__(self, robot_id, joint_id, P, D):
        self.robot_id = robot_id
        self.joint_id = joint_id
        self.P = P
        self.D = D

    def control(self, p_des, v_des):
        pos, vel, *other = p.getJointState(self.robot_id, self.joint_id)
        torque = self.P * (p_des - pos) + self.D * (v_des - vel)
        p.setJointMotorControl2(self.robot_id, self.joint_id, controlMode=p.TORQUE_CONTROL, force=torque, positionGain=0., velocityGain=0.)
        return torque

    def set_gains(self, P, D):
        self.P = P
        self.D = D


def desired_state(specification, time_vector, optimized_sequence):

    def desired_state_eval(t):
        closest_idx = np.argmin(abs(time_vector - t))
        # Determine interval
        if time_vector[closest_idx] > t:
            t1_idx = max(closest_idx - 1, 0)
            t2_idx = closest_idx
        else:
            t1_idx = closest_idx
            t2_idx = min(closest_idx + 1, len(time_vector) - 1)

        if specification == "POSITION":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_posture.joint_positions
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_posture.joint_positions
        elif specification == "VELOCITY":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_velocity.joint_velocities
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_velocity.joint_velocities

        delta_t = t - time_vector[t1_idx]
        if t2_idx <= 0:
            state = state_1
        elif t1_idx >= len(time_vector) - 1:
            state = state_1
        else:
            # linearly interpolate between states
            state = (state_2 - state_1) / (time_vector[t2_idx] - time_vector[t1_idx]) * delta_t + state_1

        return state

    return desired_state_eval


class MotionExecutor():

    def __init__(self, optimized_sequence, time_vector):
        self.optimized_sequence = optimized_sequence
        self.time_vector = time_vector

        # physicsClient = p.connect(p.GUI)
        physicsClient = p.connect(p.DIRECT)

        urdf_base_string = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        planeId = p.loadURDF(urdf_base_string + "/urdf/plane_with_restitution.urdf")
        cubeStartPos = [0,0,0.30]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robotId = p.loadURDF(urdf_base_string + "/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)

        useRealTimeSimulation = False

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)
        print("Number of joints={}".format(num_joints))

        urdf = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/urdf/quadruped.urdf')
        self.robot = QuadrupedWrapper(urdf)

        self.joint_map = {}
        self.joint_map_inv = {}
        for ji in range(num_joints):
            self.joint_map[p.getJointInfo(self.robotId, ji)[1].decode('UTF-8')] = ji
            self.joint_map_inv[ji] = p.getJointInfo(self.robotId, ji)[1].decode('UTF-8')
            p.changeDynamics(self.robotId,ji,linearDamping=.04, angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        # print(self.joint_map)

        FL_HFE = self.joint_map['FL_HFE']
        FL_KFE = self.joint_map['FL_KFE']
        FR_HFE = self.joint_map['FR_HFE']
        FR_KFE = self.joint_map['FR_KFE']
        BL_HFE = self.joint_map['BL_HFE']
        BL_KFE = self.joint_map['BL_KFE']
        BR_HFE = self.joint_map['BR_HFE']
        BR_KFE = self.joint_map['BR_KFE']
        self.controlled_joints = [
            FL_HFE, FL_KFE, FR_HFE, FR_KFE, BL_HFE, BL_KFE, BR_HFE, BR_KFE
        ]

        pinocchio_names = [name for name in self.robot.model.names[2:]]
        self.pin_joint_map = {}
        self.pin_joint_map_inv = {}
        for i, name in enumerate(pinocchio_names):
            self.pin_joint_map[name] = i
            self.pin_joint_map_inv[i] = name

        self.init_config = optimized_sequence.kinematics_states[0].robot_posture.joint_positions

        for i, id in enumerate(self.controlled_joints):
            des_pos = self.init_config[i]
            p.resetJointState(self.robotId, id, des_pos, 0)

        id_joints = list(range(num_joints))
        p.setJointMotorControlArray(self.robotId, id_joints, p.VELOCITY_CONTROL, forces=np.zeros(num_joints))

        if (useRealTimeSimulation):
            p.setRealTimeSimulation(1)

        # Enable force measurements at the contact points.
        id_end_joints = [self.joint_map[name] for name in ['FL_END', 'FR_END', 'BL_END', 'BR_END']]
        for id in id_end_joints:
            p.enableJointForceTorqueSensor(self.robotId, id, enableSensor=True)

        # self.sim = Simulator(self.robotId, self.robot,
        #                      ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'BL_HFE', 'BL_KFE', 'BR_HFE', 'BR_KFE'],
        #                      ['FL_END', 'FR_END', 'BL_END', 'BR_END'])

        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(1e-3, numSubSteps=1)
        print(p.getPhysicsEngineParameters())

    def bullet2pin(self, joint_id):
        return self.pin_joint_map[self.joint_map_inv[joint_id]]

    def calculate_actual_trajectories(self, num_loops, t_vec, joint_configurations, base_states):
        print("Determining actual COM, LMOM and AMOM trajectories...")
        com_trajectory = np.zeros((num_loops, 3))
        lmom_trajectory = np.zeros((num_loops, 3))
        amom_trajectory = np.zeros((num_loops, 3))

        q_new = self.robot.q.copy()
        q_previous = self.robot.q.copy()
        num_uncontrolled_joints = q_new.shape[0] - len(self.controlled_joints)

        for loop in range(num_loops):
            q_new[:num_uncontrolled_joints] = np.reshape(base_states[loop], (num_uncontrolled_joints, 1))

            q_offset_index = num_uncontrolled_joints - 2
            for i in range(len(self.controlled_joints)):
                joint_state = joint_configurations[loop, i]
                bullet_name = self.joint_map_inv[self.controlled_joints[i]]
                joint_id = self.robot.model.getJointId(bullet_name)

                q_new[q_offset_index + joint_id, 0] = joint_state

            self.robot.set_configuration(q_new)

            if loop == 0:
                q_previous = q_new.copy()
                q_dot = self.robot.get_difference(q_previous, q_new)
            else:
                q_dot = self.robot.get_difference(q_previous, q_new) / (t_vec[loop] - t_vec[loop - 1]) 
            
            self.robot.centroidalMomentum(q_new, q_dot)
            q_previous = q_new.copy()
            com_trajectory[loop, :] = np.squeeze(np.array(self.robot.com(q_new)), 1)
            lmom_trajectory[loop, :] = np.squeeze(self.robot.data.hg.vector[:3], 1)
            amom_trajectory[loop, :] = np.squeeze(self.robot.data.hg.vector[3:], 1)

        print("...Done.")
        return com_trajectory, lmom_trajectory, amom_trajectory

    def execute_motion(self):
        controllers = [PDController(self.robotId, id, 5.0, 0.2) for id in self.controlled_joints]

        num_uncontrolled_joints = 6
        torques = np.zeros((num_uncontrolled_joints + len(self.controlled_joints)))

        # Apply gains to reach steady state
        loop = 0
        try:
            while loop < 2000:
                torques = []

                for i, c in enumerate(controllers):
                    pinocchio_index = self.bullet2pin(self.controlled_joints[i])
                    torques.append(c.control(self.init_config[pinocchio_index], 0.0))

                # self.sim.send_joint_command(torques)
                # self.sim.step()

                p.stepSimulation()
                # sleep(0.001)

                loop += 1

        except KeyboardInterrupt:
            print("Keyboard interrupt")

        desired_pos = desired_state("POSITION", self.time_vector, self.optimized_sequence)
        desired_vel = desired_state("VELOCITY", self.time_vector, self.optimized_sequence)

        time_horizon = 4.0
        max_num_iterations = int(time_horizon * 1000)

        desired_pos_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        desired_vel_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        actual_pos_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        actual_vel_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        base_states = np.zeros((max_num_iterations, 7))

        t_vec = np.zeros((max_num_iterations))

        loop = 0
        # t_0 = time()

        # Apply gains for trajectory tracking
        try:
            print("Executing motion...")
            # while time() - t_0 < time_horizon and loop < max_num_iterations:
            # t_0 = time()
            while loop < max_num_iterations:
                t = loop / 1e3
                # t = time() - t_0

                torques = []
                des_pos = desired_pos(t)
                des_vel = desired_vel(t)

                for i, c in enumerate(controllers):
                    pinocchio_index = self.bullet2pin(self.controlled_joints[i])

                    torques.append(c.control(des_pos[pinocchio_index], des_vel[pinocchio_index]))
                    actual_pos_arr[loop, i], actual_vel_arr[loop, i], _, _ = p.getJointState(self.robotId, self.controlled_joints[i]) 
                
                # self.sim.send_joint_command(torques)
                
                desired_pos_arr[loop, :] = des_pos
                desired_vel_arr[loop, :] = des_vel
                base_state_and_orientation = p.getBasePositionAndOrientation(self.robotId)
                base_states[loop, :3] = base_state_and_orientation[0]
                base_states[loop, 3:] = base_state_and_orientation[1]
                t_vec[loop] = t

                p.stepSimulation()
                # sleep(0.001)

                loop += 1

            print("...Finished execution.")

            actual_com, actual_lmom, actual_amom = self.calculate_actual_trajectories(loop, t_vec, actual_pos_arr, base_states)

            desired_com = np.zeros((len(self.time_vector), 3))
            desired_lmom = np.zeros((len(self.time_vector), 3))
            desired_amom = np.zeros((len(self.time_vector), 3))
            for t in range(len(self.time_vector)):
                desired_com[t, :] = self.optimized_sequence.kinematics_states[t].com
                desired_lmom[t, :] = self.optimized_sequence.kinematics_states[t].lmom
                desired_amom[t, :] = self.optimized_sequence.kinematics_states[t].amom

            # Apply delta to desired_com, because of different initial positions
            desired_com += actual_com[0, :] - desired_com[0, :] 

            actual_trajectories = {"joint_configs": actual_pos_arr, "joint_velocities": actual_vel_arr,
                                   "COM": actual_com, "LMOM": actual_lmom, "AMOM": actual_amom}
            desired_trajectories = {"joint_configs": desired_pos_arr, "joint_velocities": desired_vel_arr, 
                                    "COM": desired_com, "LMOM": desired_lmom, "AMOM": desired_amom}

            self.plot_execution(t_vec, loop, desired_trajectories, actual_trajectories)

        except KeyboardInterrupt:
            print("Keyboard interrupt") 


    def plot_execution(self, t_vec, used_loops, desired_trajectories, actual_trajectories):
        import matplotlib.pyplot as plt

        joint_types = self.robot.joints_list.copy() 
        del joint_types[joint_types.index("END")]
        joint_positions = self.robot.effs

        joint_states = ["joint_configs", "joint_velocities"]

        for joint_state in joint_states:
            if joint_state == "joint_configs":
                specification = " configuration"
            else:
                specification = " velocity"
            for joint_type in joint_types:
                fig, axes = plt.subplots(2, 2, sharex='col')
                i = 0
                for joint_pos in joint_positions:
                    joint_name = joint_pos + "_" + joint_type
                    pin_index = self.pin_joint_map[joint_name]
                    joint_index = np.where(np.array(self.controlled_joints) == self.joint_map[joint_name])[0][0]

                    idx_1, idx_2 = np.unravel_index([i], (2, 2))
                    ax = axes[idx_1[0], idx_2[0]]
                    ax.plot(t_vec[:used_loops], desired_trajectories[joint_state][:used_loops, pin_index], "r", label="Desired")
                    ax.plot(t_vec[:used_loops], actual_trajectories[joint_state][:used_loops, joint_index], "b", label="Actual")
                    ax.set_title(joint_pos)
                    ax.legend()
                    if joint_state == "joint_configs":
                        ax.set_ylabel("theta [rad]")
                    else:
                        ax.set_ylabel("theta_dot [rad / s]")
                    i += 1

                axes[1, 0].set_xlabel("t [s]")
                axes[1, 1].set_xlabel("t [s]")
                fig.suptitle(joint_type + specification)

        momentums = ["LMOM", "AMOM", "COM"]
        coords = ["y", "z"]

        for momentum in momentums:
            fig, axes = plt.subplots(2, 1, sharex='col')

            for i, coord in enumerate(coords):
                coord_index = i + 1
                axes[i].plot(self.time_vector, desired_trajectories[momentum][:, coord_index], "r", label="Desired " + momentum)
                axes[i].plot(t_vec[:used_loops], actual_trajectories[momentum][:used_loops, coord_index], "b", label="Actual " + momentum)
                axes[i].legend()
                if momentum == "COM":
                    axes[i].set_ylabel(coord + " [m]")
                elif momentum == "LMOM":
                    axes[i].set_ylabel("p_" + coord + " [kg * m / s]")
                elif momentum == "AMOM":
                    axes[i].set_ylabel("L_" + coord + " [kg * m^2 / s]")
                else:
                    raise ValueError("Momentum %s is not available." %momentum)

            axes[-1].set_xlabel("t [s]")
            fig.suptitle(momentum) 

        plt.show()
