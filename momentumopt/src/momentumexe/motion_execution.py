import os
import numpy as np
from time import sleep, time

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from pymomentum import *

from quadruped.quadruped_wrapper import QuadrupedWrapper
from quadruped.simulator import Simulator
from momentumopt.kinoptpy.utils import isfloat


np.set_printoptions(precision=2, suppress=True, linewidth=140)


class PDController(object):
    def __init__(self, robot_id, joint_id, P, D):
        self.robot_id = robot_id
        self.joint_id = joint_id
        self.P = P
        self.D = D

    def control(self, p_des, v_des):
        pos, vel, other = p.getJointState(self.robot_id, self.joint_id)
        torque = self.P * (p_des - pos) + self.D * (v_des - vel)
        p.setJointMotorControl2(self.robot_id, self.joint_id, controlMode=p.TORQUE_CONTROL, force=torque, positionGain=0., velocityGain=0.)
        return torque

    def set_gains(self, P, D):
        self.P = P
        self.D = D


def desired_state(specification, time_vector, optimized_sequence=None, dynamics_feedback=None,
                    optimized_dyn_plan=None):
    if optimized_sequence is None and dynamics_feedback is None:
        raise ValueError("Specify desired positions, velocities or dynamic feedback gains.")

    def eff_force_vector(dynamic_state):
        """ Combines the forces at each endeffector into a single vector. """
        return np.hstack([
            dynamic_state.effForce(eff_id) for eff_id in range(dynamic_state.effNum())
        ])

    def desired_state_eval(t):
        closest_idx = np.argmin(abs(time_vector - t))
        # Determine interval
        if time_vector[closest_idx] > t:
            t1_idx = max(closest_idx - 1, 0)
            t2_idx = closest_idx
        else:
            t1_idx = closest_idx
            t2_idx = min(closest_idx + 1, len(time_vector) - 1)

        state_1 = None
        state_2 = None

        if specification == "POSITION":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_posture.joint_positions
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_posture.joint_positions
        elif specification == "VELOCITY":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_velocity.joint_velocities
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_velocity.joint_velocities
        elif specification == "GENERALIZED_POSITION":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_posture.generalized_joint_positions
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_posture.generalized_joint_positions
        elif specification == "GENERALIZED_VELOCITY":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_velocity.generalized_joint_velocities
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_velocity.generalized_joint_velocities
        elif specification == "GENERALIZED_ACCELERATION":
            state_1 = optimized_sequence.kinematics_states[t1_idx].robot_acceleration.generalized_joint_accelerations
            state_2 = optimized_sequence.kinematics_states[t2_idx].robot_acceleration.generalized_joint_accelerations
        elif specification == 'FORCES':
            state_1 = eff_force_vector(optimized_dyn_plan.dynamics_states[t1_idx])
            state_2 = eff_force_vector(optimized_dyn_plan.dynamics_states[t2_idx])
        elif specification == "COM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].com
            state_2 = optimized_sequence.kinematics_states[t2_idx].com
        elif specification == "LMOM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].lmom
            state_2 = optimized_sequence.kinematics_states[t2_idx].lmom
        elif specification == "AMOM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].amom
            state_2 = optimized_sequence.kinematics_states[t2_idx].amom
        elif specification == "DYN_FEEDBACK":
            state_1 = dynamics_feedback.forceGain(t1_idx)
            state_2 = dynamics_feedback.forceGain(t2_idx)

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

def interpolate(specification, time_vector, optimized_motion_eff=None, optimized_sequence=None, dynamics_feedback=None, optimized_dyn_plan=None):
    ## for impedance controller
    ## interpolates end_eff velocity and positions to bring it to sample rate of 1khz

    if optimized_motion_eff is None and dynamics_feedback is None and optimized_sequence is None:
        raise ValueError("Specify desired positions, velocities or dynamic feedback gains.")

    def eff_force_vector(dynamic_state):
        """ Combines the forces at each endeffector into a single vector. """
        return np.hstack([
            dynamic_state.effForce(eff_id) for eff_id in range(dynamic_state.effNum())
        ])

    def desired_state_eval(t):
        closest_idx = np.argmin(abs(time_vector - t))
        # Determine interval
        if time_vector[closest_idx] > t:
            t1_idx = max(closest_idx - 1, 0)
            t2_idx = closest_idx
        else:
            t1_idx = closest_idx
            t2_idx = min(closest_idx + 1, len(time_vector) - 1)

        state_1 = None
        state_2 = None

        if specification == "POSITION":
            state_1 = optimized_motion_eff["trajectory_wrt_base"][t1_idx]
            state_2 = optimized_motion_eff["trajectory_wrt_base"][t2_idx]
        elif specification == "VELOCITY":
            state_1 = optimized_motion_eff["velocity_wrt_base"][t1_idx]
            state_2 = optimized_motion_eff["velocity_wrt_base"][t2_idx]
        elif specification == "POSITION_ABSOLUTE":
            state_1 = optimized_motion_eff["trajectory"][t1_idx]
            state_2 = optimized_motion_eff["trajectory"][t2_idx]
        elif specification == "VELOCITY_ABSOLUTE":
            state_1 = optimized_motion_eff["velocity"][t1_idx]
            state_2 = optimized_motion_eff["velocity"][t2_idx]

        elif specification == "COM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].com
            state_2 = optimized_sequence.kinematics_states[t2_idx].com
        elif specification == "LMOM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].lmom
            state_2 = optimized_sequence.kinematics_states[t2_idx].lmom
        elif specification == "AMOM":
            state_1 = optimized_sequence.kinematics_states[t1_idx].amom
            state_2 = optimized_sequence.kinematics_states[t2_idx].amom
        elif specification == "DYN_FEEDBACK":
            state_1 = dynamics_feedback.forceGain(t1_idx)
            state_2 = dynamics_feedback.forceGain(t2_idx)
        elif specification == 'FORCES':
            state_1 = eff_force_vector(optimized_dyn_plan.dynamics_states[t1_idx])
            state_2 = eff_force_vector(optimized_dyn_plan.dynamics_states[t2_idx])

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

def query_gain_from_user(K, gain_str, entered_joint_id):
    gain = ""
    while not isfloat(gain):
        print("Enter " + gain_str + "-gain or press Enter to keep current gain: ")
        gain = input()
        if isfloat(gain):
            K[entered_joint_id] = float(gain)
        elif gain == "":
            break
        else:
            print("Entered incorrect input. Please try again.")

    return K


class MotionSimulator(object):
    def __init__(self, floor_height=0.3, bullet_direct=False):
        self.controlled_joints = 6

        self.tau_min = - 2.0
        self.tau_max = 2.0

        if bullet_direct:
            physicsClient = p.connect(p.DIRECT)
        else:
            physicsClient = p.connect(p.GUI)

        urdf_base_string = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        planeId = p.loadURDF(urdf_base_string + "/urdf/plane_with_restitution.urdf")
        cubeStartPos = [0, 0, floor_height]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robotId = p.loadURDF(urdf_base_string + "/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)

        useRealTimeSimulation = False

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)
        print("Number of joints={}".format(num_joints))

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04, angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(1e-3, numSubSteps=1)
        print(p.getPhysicsEngineParameters())

        # Create the pinocchio robot.
        urdf = str(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/urdf/quadruped.urdf')
        self.robot = QuadrupedWrapper(urdf)

        self.controlled_joints = ['BL_HFE', 'BL_KFE', 'BR_HFE', 'BR_KFE', 'FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE']

        # Create the simulator for easier mapping between
        self.sim = Simulator(self.robotId, self.robot,
            self.controlled_joints,
            ['BL_END', 'BR_END', 'FL_END', 'FR_END', ]
        )

class MotionExecutor(MotionSimulator):

    def __init__(self, optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector):
        super(MotionExecutor, self).__init__()

        self.optimized_kin_plan = optimized_kin_plan
        self.optimized_dyn_plan = optimized_dyn_plan
        self.dynamics_feedback = dynamics_feedback
        self.time_vector = time_vector
        self.planner_setting = planner_setting


        q, dq = self.sim.get_state()
        q[7:] = self.optimized_kin_plan.kinematics_states[0].robot_posture.joint_positions.reshape((-1, 1))
        self.init_config = q.copy()

        self.sim.reset_state(q, dq)

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
            q_new[num_uncontrolled_joints:] = joint_configurations[loop].reshape((-1, 1))

            self.robot.set_configuration(q_new)

            if loop == 0:
                q_previous = q_new.copy()
                q_dot = self.robot.get_difference(q_previous, q_new)
            else:
                q_dot = self.robot.get_difference(q_previous, q_new) / (t_vec[loop] - t_vec[loop - 1])

            self.robot.robot.centroidalMomentum(q_new, q_dot)
            q_previous = q_new.copy()
            com_trajectory[loop, :] = np.squeeze(np.array(self.robot.robot.com(q_new)), 1)
            lmom_trajectory[loop, :] = np.squeeze(self.robot.data.hg.vector[:3], 1)
            amom_trajectory[loop, :] = np.squeeze(self.robot.data.hg.vector[3:], 1)

        print("...Done.")
        return com_trajectory, lmom_trajectory, amom_trajectory

    def calculate_momentum(self, delta_t, joint_configuration, base_state):
        q_new = self.robot.q.copy()
        q_previous = self.robot.q.copy()
        num_uncontrolled_joints = q_new.shape[0] - len(self.controlled_joints)

        q_new[:num_uncontrolled_joints] = np.reshape(base_state, (num_uncontrolled_joints, 1))
        q_new[num_uncontrolled_joints:] = joint_configuration.reshape((-1, 1))

        self.robot.set_configuration(q_new)

        if delta_t < 1e-8:
            q_previous = q_new.copy()
            q_dot = self.robot.get_difference(q_previous, q_new)
        else:
            q_dot = self.robot.get_difference(q_previous, q_new) / delta_t

        self.robot.robot.centroidalMomentum(q_new, q_dot)
        com = np.squeeze(np.array(self.robot.robot.com(q_new)), 1)
        lmom = np.squeeze(np.array(self.robot.data.hg.vector[:3]), 1)
        amom = np.squeeze(np.array(self.robot.data.hg.vector[3:]), 1)

        return com, lmom, amom

    def limit_torques(self, torque):
        torque[torque < self.tau_min] = self.tau_min
        torque[torque > self.tau_max] = self.tau_max
        return torque

    def execute_motion(self, plotting=False, tune_online=False):
        sim = self.sim
        P, D = 10. * np.ones(8), 0.3 * np.ones(8)

        for i in range(8):
            if "HFE" in self.robot.model.names[int(sim.pinocchio_joint_ids[i])]:
                D[i] = 0.5

            if "FR" in self.robot.model.names[int(sim.pinocchio_joint_ids[i])] or "FL" in self.robot.model.names[int(sim.pinocchio_joint_ids[i])]:
                D[i] = 0.5

        num_uncontrolled_joints = 6

        # Apply gains to reach steady state
        loop = 0
        try:
            while loop < 2000:
                q, dq = sim.get_state()

                ptau = np.diag(P) * se3.difference(self.robot.model, q, self.init_config)[6:]
                ptau += np.diag(D) * -dq[6:]
                self.limit_torques(ptau)

                sim.send_joint_command(ptau)
                sim.step()

                loop += 1

        except KeyboardInterrupt:
            print("Keyboard interrupt")

        desired_pos = desired_state("POSITION", self.time_vector, optimized_sequence=self.optimized_kin_plan)
        desired_vel = desired_state("VELOCITY", self.time_vector, optimized_sequence=self.optimized_kin_plan)
        desired_com_func = desired_state("COM", self.time_vector, optimized_sequence=self.optimized_kin_plan)
        desired_lmom_func = desired_state("LMOM", self.time_vector, optimized_sequence=self.optimized_kin_plan)
        desired_amom_func = desired_state("AMOM", self.time_vector, optimized_sequence=self.optimized_kin_plan)
        if not self.dynamics_feedback is None:
            dyn_feedback = desired_state("DYN_FEEDBACK", self.time_vector, dynamics_feedback=self.dynamics_feedback)

        time_horizon = 4.0
        max_num_iterations = int(time_horizon * 1000)

        desired_pos_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        desired_vel_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        actual_pos_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        actual_vel_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        base_states = np.zeros((max_num_iterations, 7))
        tau_arr = np.zeros((max_num_iterations, len(self.controlled_joints)))
        forces_arr = np.zeros((max_num_iterations, len(sim.pinocchio_endeff_ids), 6))

        t_vec = np.zeros((max_num_iterations))

        # t_0 = time()

        robot_weight = self.planner_setting.get(PlannerDoubleParam_RobotWeight)

        jacobians_eff = {}
        for eff in self.robot.effs:
            for joint in self.robot.joints_list:
                joint_identifier = eff + "_" + joint
                jacobians_eff[joint_identifier] = self.robot.get_jacobian(joint_identifier, "TRANSLATION")

        # Apply gains for trajectory tracking
        try:
            print("Executing motion...")
            executing = True
            # t_0 = None
            # t_1 = None

            force_offset = 0

            while executing:
                loop = 0
                time_id = 0

                swing_times = {}
                for eff in self.robot.effs:
                    swing_times[eff] = []

                while loop < max_num_iterations:
                    t = loop / 1e3
                    if t > self.time_vector[time_id]:
                        time_id += 1

                    des_pos = desired_pos(t)
                    des_vel = desired_vel(t)

                    q, dq = sim.get_state()
                    frame_ids, forces = sim.get_force()

                    q_des = q.copy()
                    q_des[7:] = des_pos.reshape((-1, 1))
                    dq_des = dq.copy()
                    dq_des[6:] = des_vel.reshape((-1, 1))

                    ptau = np.diag(P) * se3.difference(self.robot.model, q, q_des)[6:]
                    ptau += np.diag(D) * (dq_des - dq)[6:]

                    planned_force = np.zeros((3 * len(self.robot.effs)))
                    jacobians_effs = np.zeros((3 * len(self.robot.effs), self.robot.robot.nv))
                    for eff_id, eff in enumerate(self.robot.effs):
                        eff = eff + "_END"
                        force = self.optimized_dyn_plan.dynamics_states[time_id].effForce(eff_id) * robot_weight
                        # force = self.optimized_dyn_plan.dynamics_states[min(time_id - force_offset, len(self.time_vector) - 1)].effForce(eff_id) * robot_weight
                        # force = self.optimized_dyn_plan.dynamics_states[max(time_id - force_offset, 0)].effForce(eff_id) * robot_weight
                        planned_force[eff_id * 3 : eff_id * 3 + 3] = force
                        jacobians_effs[eff_id * 3 : eff_id * 3 + 3, :] = jacobians_eff[eff]()

                    if self.dynamics_feedback is None:
                        use_dyn_feedback = False
                    else:
                        use_dyn_feedback = True

                    if use_dyn_feedback:
                        if loop == 0:
                            delta_t = 0
                        else:
                            delta_t = t - t_vec[-1]
                        joint_configuration = q[7:].reshape((-1))
                        base_state = np.zeros((7))
                        base_state[:3], base_state[3:] = p.getBasePositionAndOrientation(self.robotId)
                        com, lmom, amom = self.calculate_momentum(delta_t, joint_configuration, base_state)
                        desired_momentum = np.hstack((desired_com_func(t), desired_lmom_func(t), desired_amom_func(t)))
                        current_momentum = np.hstack((com, lmom, amom))
                        delta_momentum = current_momentum - desired_momentum
                        delta_force = np.dot(dyn_feedback(t), delta_momentum)
                    else:
                        delta_force = np.zeros_like(planned_force)

                    ptau += np.reshape(np.transpose(np.dot(np.transpose(jacobians_effs), planned_force + delta_force))[6:], (ptau.shape[0], 1))

                    self.limit_torques(ptau)

                    sim.send_joint_command(ptau)

                    desired_pos_arr[loop, :] = des_pos
                    desired_vel_arr[loop, :] = des_vel
                    actual_pos_arr[loop, :] = q[7:].reshape((-1))
                    actual_vel_arr[loop, :] = dq[6:].reshape((-1))
                    base_state_and_orientation = p.getBasePositionAndOrientation(self.robotId)
                    base_states[loop, :3] = base_state_and_orientation[0]
                    base_states[loop, 3:] = base_state_and_orientation[1]
                    t_vec[loop] = t
                    tau_arr[loop, :] = np.squeeze(np.array(ptau), 1)
                    for cnt_idx in range(len(forces)):
                        endeff_id = np.where(np.array(sim.pinocchio_endeff_ids) == frame_ids[cnt_idx])[0][0]
                        forces_arr[loop, endeff_id, :] = forces[cnt_idx]
                        robot_endeff = self.robot.effs[endeff_id]

                        # Determine swing times
                        if np.sum(np.abs(forces[cnt_idx])) < 1e-3:
                            if len(swing_times[robot_endeff]) == 0:
                                swing_times[robot_endeff] = [[t]]
                            elif len(swing_times[robot_endeff][-1]) == 2:
                                swing_times[robot_endeff].append([t])
                        else:
                            if len(swing_times[robot_endeff]) > 0:
                                if len(swing_times[robot_endeff][-1]) == 1:
                                    swing_times[robot_endeff][-1].append(t)

                    sim.step()
                    # sleep(0.001)

                    loop += 1

                # force_offset += 1
                actual_com, actual_lmom, actual_amom = self.calculate_actual_trajectories(loop, t_vec, actual_pos_arr, base_states)

                desired_com = np.zeros((len(self.time_vector), 3))
                desired_lmom = np.zeros((len(self.time_vector), 3))
                desired_amom = np.zeros((len(self.time_vector), 3))
                for t in range(len(self.time_vector)):
                    desired_com[t, :] = self.optimized_kin_plan.kinematics_states[t].com
                    desired_lmom[t, :] = self.optimized_kin_plan.kinematics_states[t].lmom
                    desired_amom[t, :] = self.optimized_kin_plan.kinematics_states[t].amom

                # Apply delta to desired_com, because of different initial positions
                desired_com += actual_com[0, :] - desired_com[0, :]

                actual_trajectories = {"joint_configs": actual_pos_arr, "joint_velocities": actual_vel_arr,
                                       "COM": actual_com, "LMOM": actual_lmom, "AMOM": actual_amom}
                desired_trajectories = {"joint_configs": desired_pos_arr, "joint_velocities": desired_vel_arr,
                                        "COM": desired_com, "LMOM": desired_lmom, "AMOM": desired_amom}

                if plotting:
                    self.plot_execution(t_vec, loop, desired_trajectories, actual_trajectories, swing_times)
                    self.plot_torques(t_vec, loop, tau_arr, swing_times)
                    self.plot_forces(t_vec, loop, forces_arr, swing_times)

                if tune_online:
                    P, D = self.tunePD(P, D)
                else:
                    executing = False
                    # pass

            print("...Finished execution.")

        except KeyboardInterrupt:
            print("Keyboard interrupt")

    def print_joint_gains(self, P, D):
        print("Which joint do you want to tune?")
        print("ID  |   Joint    |     P    |    D  ")
        print("-------------------------------------")
        for i in range(len(self.controlled_joints)):
            print(i, "  |  ", self.robot.model.names[int(self.sim.pinocchio_joint_ids[i])], "  |  ", P[i], "  |  ", D[i])
        print("Enter the joint id (0-7), enter 'stop' to execute motion:")

    def tunePD(self, P, D):
        print("CONTROLLER TUNING")
        tuning = True

        while tuning:
            self.print_joint_gains(P, D)
            entered_joint_id = input()
            incorrect_input = True
            if entered_joint_id == "stop":
                incorrect_input = False
                tuning = False
                continue

            while incorrect_input and tuning:
                if entered_joint_id.isdigit():
                    entered_joint_id = int(entered_joint_id)
                    if entered_joint_id >= 0 and entered_joint_id < len(self.controlled_joints):
                        incorrect_input = False

                if incorrect_input:
                    print("Entered incorrect input. Please try again.")
                    self.print_joint_gains(P, D)
                    entered_joint_id = input()
                    if entered_joint_id == "stop":
                        tuning = False

            if tuning:
                joint_name = self.robot.model.names[int(self.sim.pinocchio_joint_ids[entered_joint_id])]
                print("Tuning joint ", joint_name, "...")

                P = query_gain_from_user(P, "P", entered_joint_id)
                D = query_gain_from_user(D, "D", entered_joint_id)

        return P, D

    def plot_execution(self, t_vec, used_loops, desired_trajectories, actual_trajectories, swing_times):
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
                    joint_index = self.sim.joint_names.index(joint_name)

                    idx_1, idx_2 = np.unravel_index([i], (2, 2))
                    ax = axes[idx_1[0], idx_2[0]]
                    ax.plot(t_vec[:used_loops], desired_trajectories[joint_state][:used_loops, joint_index], "r", label="Desired")
                    ax.plot(t_vec[:used_loops], actual_trajectories[joint_state][:used_loops, joint_index], "b", label="Actual")

                    for eff in self.robot.effs:
                        for swing_time in swing_times[eff]:
                            t_0, t_1 = swing_time
                            ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                            ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

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
        coords = ["x", "y", "z"]

        for momentum in momentums:
            fig, axes = plt.subplots(len(coords), 1, sharex='col')

            for i, coord in enumerate(coords):
                axes[i].plot(self.time_vector, desired_trajectories[momentum][:, i], "r", label="Desired " + momentum)
                axes[i].plot(t_vec[:used_loops], actual_trajectories[momentum][:used_loops, i], "b", label="Actual " + momentum)

                for eff in self.robot.effs:
                    for swing_time in swing_times[eff]:
                        t_0, t_1 = swing_time
                        axes[i].axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                        axes[i].axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

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

    def plot_torques(self, t_vec, used_loops, torques, swing_times):
        import matplotlib.pyplot as plt

        joint_types = self.robot.joints_list.copy()
        del joint_types[joint_types.index("END")]
        joint_positions = self.robot.effs

        fig, axes = plt.subplots(2, 4, sharex='col')
        i = 0
        for joint_type in joint_types:
            for joint_pos in joint_positions:
                joint_name = joint_pos + "_" + joint_type
                joint_index = self.sim.joint_names.index(joint_name)

                idx_1, idx_2 = np.unravel_index([i], (2, 4))
                ax = axes[idx_1[0], idx_2[0]]
                ax.plot(t_vec[:used_loops], torques[:used_loops, joint_index], "r", label=joint_name)

                for eff in self.robot.effs:
                    for swing_time in swing_times[eff]:
                        t_0, t_1 = swing_time
                        ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                        ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

                ax.axhline(y=-2.0, color="k", linestyle="-")
                ax.axhline(y=2.0, color="k", linestyle="-")

                ax.set_title(joint_name)
                i += 1

        axes[0, 0].set_ylabel("tau [Nm]")
        axes[1, 0].set_ylabel("tau [Nm]")

        axes[1, 0].set_xlabel("t [s]")
        axes[1, 1].set_xlabel("t [s]")
        axes[1, 2].set_xlabel("t [s]")
        axes[1, 3].set_xlabel("t [s]")
        fig.suptitle("Joint Torques")

        plt.show()

    def plot_forces(self, t_vec, used_loops, forces, swing_times):
        import matplotlib.pyplot as plt

        joint_positions = self.robot.effs

        fig, axes = plt.subplots(2, 2, sharex='col')
        i = 0
        for joint_pos in joint_positions:
            joint_name = joint_pos + "_END"

            idx_1, idx_2 = np.unravel_index([i], (2, 2))
            ax = axes[idx_1[0], idx_2[0]]
            ax.plot(t_vec[:used_loops], - forces[:used_loops, i, 2], "r", label=joint_name)

            for eff in self.robot.effs:
                for swing_time in swing_times[eff]:
                    t_0, t_1 = swing_time
                    ax.axvline(x=t_0, color=self.robot.colors[eff], linestyle="--", alpha=0.25)
                    ax.axvline(x=t_1, color=self.robot.colors[eff], linestyle="--", alpha=0.25)

            ax.set_title(joint_name)
            i += 1

        axes[0, 0].set_ylabel("F_z [N]")
        axes[1, 0].set_ylabel("F_z [N]")

        axes[1, 0].set_xlabel("t [s]")
        axes[1, 1].set_xlabel("t [s]")

        fig.suptitle("Forces")

        plt.show()
