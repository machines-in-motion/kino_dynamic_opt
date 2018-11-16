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
        physicsClient = p.connect(p.GUI)
        # physicsClient = p.connect(p.DIRECT)
        urdf_base_string = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        planeId = p.loadURDF(urdf_base_string + "/urdf/plane_with_restitution.urdf")
        print(p.getDynamicsInfo(planeId, -1))
        cubeStartPos = [0,0,0.30]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robotId = p.loadURDF(urdf_base_string + "/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        print(cubePos)

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

        print(self.joint_map)

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

                if loop % 100 == 0:
                    print(torques)

                # self.sim.send_joint_command(torques)
                # self.sim.step()

                p.stepSimulation()

                loop += 1
                sleep(0.001)

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

        t_vec = np.zeros((max_num_iterations))

        loop = 0
        t_0 = time()

        # Apply gains for trajectory tracking
        try:
            while time() - t_0 < time_horizon or loop < max_num_iterations:
                t = time() - t_0

                torques = []
                des_pos = desired_pos(t)
                des_vel = desired_vel(t)

                for i, c in enumerate(controllers):
                    pinocchio_index = self.bullet2pin(self.controlled_joints[i])

                    torques.append(c.control(des_pos[pinocchio_index], des_vel[pinocchio_index]))
                    actual_pos_arr[loop, i], actual_vel_arr[loop, i], nothing, nothing = p.getJointState(self.robotId, self.controlled_joints[i]) 
                
                # self.sim.send_joint_command(torques)

                if loop % 100 == 0:
                    print(torques)
                
                desired_pos_arr[loop, :] = des_pos
                desired_vel_arr[loop, :] = des_vel
                t_vec[loop] = t

                # self.sim.step()

                p.stepSimulation()

                loop += 1
                sleep(0.001)

            print(loop)

            import matplotlib.pyplot as plt

            print(self.controlled_joints)
            print(self.joint_map)
            joint_index = 0
            bullet_name = self.joint_map_inv[self.controlled_joints[joint_index]]
            bullet_index = self.joint_map[bullet_name]
            print("Bullet joint name: ", bullet_name)
            print("Bullet joint id: ", self.joint_map[bullet_name])
            pin_index = self.pin_joint_map[bullet_name]
            pin_name = self.pin_joint_map_inv[pin_index]
            print("Pinocchio joint name: ", pin_name)
            print("Pinocchio joint id: ", pin_index)

            plt.plot(t_vec[:loop], desired_pos_arr[:loop, pin_index], "r", label="Desired")
            plt.plot(t_vec[:loop], actual_pos_arr[:loop, joint_index], "b", label="Actual")
            plt.title(bullet_name)
            plt.legend()
            plt.ylabel("theta [rad]")
            plt.xlabel("t [s]")

            plt.show()

        except KeyboardInterrupt:
            print("Keyboard interrupt") 
