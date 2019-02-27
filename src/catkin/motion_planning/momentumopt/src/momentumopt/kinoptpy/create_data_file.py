import numpy as np

from pymomentum import *

from src.momentumexe.motion_execution import desired_state, interpolate

sample_frequency = 1000 # 1kHz

def create_file(time_vector, optimized_sequence):
    desired_pos = desired_state("POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_vel = desired_state("VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    # TODO: Desired forces
    # desired_forces = desired_state("FORCES", time_vector, optimized_sequence=optimized_sequence)

    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.0:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    num_points = max_time * sample_frequency

    using_quadruped = True

    if using_quadruped:
        des_positions = np.zeros((num_points, 9))
        des_velocities = np.zeros((num_points, 9))

        for i in range(num_points):
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))

        np.savetxt("quadruped_positions.dat", des_positions)
        np.savetxt("quadruped_velocities.dat", des_velocities)
    else:  # using teststand
        des_positions = np.zeros((num_points, 3))
        des_velocities = np.zeros((num_points, 3))

        for i in range(num_points):
            # Only using the motion of the BL leg with the joints BL_HFE and BL_KFE
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)[:2]))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)[:2]))

        np.savetxt("teststand_positions.dat", des_positions)
        np.savetxt("teststand_velocities.dat", des_velocities)



def create_trajectory_file_impedance(time_vector, optimized_motion_eff, optimized_sequence):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    # TODO: Desired forces
    # desired_forces = desired_state("FORCES", time_vector, optimized_sequence=optimized_sequence)

    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.0:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    num_points = max_time * sample_frequency

    using_quadruped = True

    if using_quadruped:
        des_positions = np.zeros((num_points, 13))
        des_velocities = np.zeros((num_points, 13))

        for i in range(num_points):
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))

        #print(desired_pos)
        np.savetxt("quadruped_positions_eff.dat", des_positions)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities)
    # else:  # using teststand
    #     des_positions = np.zeros((num_points, 3))
    #     des_velocities = np.zeros((num_points, 3))
    #
    #     for i in range(num_points):
    #         # Only using the motion of the BL leg with the joints BL_HFE and BL_KFE
    #         des_positions[i, :] = np.hstack((i, desired_pos(i / sample_frequency)[:2]))
    #         des_velocities[i, :] = np.hstack((i, desired_vel(i / sample_frequency)[:2]))
    #
    #     np.savetxt("teststand_positions.dat", des_positions)
    #     np.savetxt("teststand_velocities.dat", des_velocities)
