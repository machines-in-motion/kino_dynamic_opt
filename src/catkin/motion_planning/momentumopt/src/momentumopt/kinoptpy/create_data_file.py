import numpy as np

from pymomentum import *

from momentumexe.motion_execution import desired_state, interpolate

sample_frequency = 1000 # 1kHz

def create_file(time_vector, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    desired_pos = desired_state("POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_vel = desired_state("VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_pos = desired_state("GENERALIZED_POSITION", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_vel = desired_state("GENERALIZED_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_gen_acc = desired_state("GENERALIZED_ACCELERATION", time_vector, optimized_sequence=optimized_sequence)
    desired_com = desired_state("COM", time_vector, optimized_sequence=optimized_sequence)
    desired_lmom = desired_state("LMOM", time_vector, optimized_sequence=optimized_sequence)
    desired_amom = desired_state("AMOM", time_vector, optimized_sequence=optimized_sequence)
    # desired_dyn_feedback = desired_state("DYN_FEEDBACK", time_vector, optimized_sequence=optimized_sequence,
    #                                 dynamics_feedback=dynamics_feedback)
    desired_forces = desired_state("FORCES", time_vector, optimized_sequence=optimized_sequence,
                                    optimized_dyn_plan=optimized_dyn_plan)

    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.0:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    num_points = max_time * sample_frequency

    using_quadruped = True

    def dump_data(output_file, desired_fn, scale=1.):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, scale * desired_fn(i / 1e3))) for i in range(num_points)
        ]))

    if using_quadruped:
        dump_data("quadruped_positions.dat", desired_pos)
        dump_data("quadruped_velocities.dat", desired_vel)
        dump_data("quadruped_forces.dat", desired_forces, robot_weight)
        dump_data("quadruped_generalized_positions.dat", desired_gen_pos)
        dump_data("quadruped_generalized_velocities.dat", desired_gen_vel)
        dump_data("quadruped_generalized_acceleration.dat", desired_gen_acc)
        dump_data("quadruped_com.dat", desired_com)
        dump_data("quadruped_lmom.dat", desired_lmom)
        dump_data("quadruped_amom.dat", desired_amom)
        # dump_data("quadruped_dyn_feedback.dat", desired_dyn_feedback)
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
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    # TODO: Desired forces
    # desired_forces = desired_state("FORCES", time_vector, optimized_sequence=optimized_sequence)

    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.0:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    print("max_time:" , max_time)
    num_points = max_time * sample_frequency

    using_quadruped = True

    if using_quadruped:
        des_positions = np.zeros((num_points, 13))
        des_velocities = np.zeros((num_points, 13))
        des_com = np.zeros((num_points, 4))

        for i in range(num_points):
            ## making des_pos and des_vel a 6d vector
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_com[i, :] = np.hstack((i, desired_com(i / 1e3)))



        swp_com = des_com[: ,1]
        des_com[: ,1] = des_com[: ,2]
        des_com[: ,2] = swp_com

        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))

        print("swapping x and y to match with current Configuration")
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                swp_pos = des_positions_final[i][6*eff]
                des_positions_final[i][6*eff] = des_positions_final[i][6*eff+1]
                des_positions_final[i][6*eff+1] = swp_pos

                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                swp_vel = des_velocities_final[i][6*eff]
                des_velocities_final[i][6*eff] = des_velocities_final[i][6*eff+1]
                des_velocities_final[i][6*eff+1] = swp_vel



        #print(desired_pos)
        print("saving trajectories....")
        np.savetxt("quadruped_positions_eff.dat", des_positions_final)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities_final)
        np.savetxt("quadruped_com.dat", des_com)
