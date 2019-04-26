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

    num_points = int(round(time_vector[-1] * sample_frequency))
    using_quadruped = True

    def dump_data(output_file, desired_fn, scale=1.):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, scale * desired_fn(i / 1e3))) for i in range(num_points)
        ]), fmt='%.8e')

    if using_quadruped:
        dump_data("quadruped_positions.dat", desired_pos)
        dump_data("quadruped_velocities.dat", desired_vel)
        dump_data("quadruped_forces.dat", desired_forces, robot_weight)
        dump_data("quadruped_generalized_positions.dat", desired_gen_pos)
        dump_data("quadruped_generalized_velocities.dat", desired_gen_vel)
        dump_data("quadruped_generalized_acceleration.dat", desired_gen_acc)
        dump_data("quadruped_com_old.dat", desired_com)
        dump_data("quadruped_lmom_old.dat", desired_lmom)
        dump_data("quadruped_amom_old.dat", desired_amom)
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

def create_lqr_impedance(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, planner_setting):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)

    max_time = 0 # time horizon in seconds

    if time_vector[-1] - int(time_vector[-1]) > 0.0:
        max_time = int(time_vector[-1]) + 1
    else:
        max_time = int(time_vector[-1])

    print("max_time:" , max_time)
    num_points = max_time * sample_frequency

    using_quadruped = True

    def dump_data(output_file, desired_fn):
        np.savetxt(output_file, np.vstack([
            np.hstack((i, desired_fn(i / 1e3))) for i in range(num_points)
        ]))

    if using_quadruped:
        des_positions = np.zeros((num_points, 13))
        des_velocities = np.zeros((num_points, 13))
        des_com = np.zeros((num_points, 4))
        des_lmom = np.zeros((num_points, 4))
        des_amom = np.zeros((num_points, 4))
        des_lqr_gains = np.zeros((num_points, 108))
        des_forces = np.zeros((num_points, 13))
        des_centroidal_forces = np.zeros((num_points, 4))
        des_centroidal_moments = np.zeros((num_points, 4))
        des_quaternion = np.zeros((num_points, 5))


        for i in range(num_points):
            ## making des_pos and des_vel a 6d vector
            ### re arranging the sequence of legs to the latest from (HR, HL, FR, FL)
            ### to (FL, FR, HL, HR) for des_pos, des_vel and forces
            des_forces[i:, ] = np.hstack((i, desired_forces(i /1e3)))
            des_centroidal_forces[i:, ] = np.hstack((i, desired_centroidal_forces(i /1e3)))
            des_centroidal_moments[i:, ] = np.hstack((i, desired_centroidal_moments(i /1e3)))
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_com[i, :] = np.hstack((i, desired_com(i / 1e3)))
            des_lmom[i, :] = np.hstack((i, desired_lmom(i / 1e3)))
            des_amom[i, :] = np.hstack((i, desired_amom(i / 1e3)))
            des_lqr_gains_tmp = desired_lqr_gains(i / 1e3)
            ## Converting lqr matrix (12 * 9) to a 108d vector
            des_lqr_gains_tmp = np.reshape(des_lqr_gains_tmp, (108,))
            des_lqr_gains[i,: ] = des_lqr_gains_tmp
            des_quaternion[i, :] = np.hstack((i, desired_quaternion(i / 1e3)))

        ## resequencing the eff sequence

        des_forces[: ,[1,2,3]], des_forces[: ,[10,11,12]] = des_forces[: ,[10,11,12]], des_forces[:, [1,2,3]].copy()
        des_forces[: ,[4,5,6]], des_forces[: ,[7,8,9]] = des_forces[: ,[7,8,9]], des_forces[:, [4,5,6]].copy()

        des_positions[: ,[1,2,3]], des_positions[: ,[10,11,12]] = des_positions[: ,[10,11,12]], des_positions[:, [1,2,3]].copy()
        des_positions[: ,[4,5,6]], des_positions[: ,[7,8,9]] = des_positions[: ,[7,8,9]], des_positions[:, [4,5,6]].copy()

        des_velocities[: ,[1,2,3]], des_velocities[: ,[10,11,12]] = des_velocities[: ,[10,11,12]], des_velocities[:, [1,2,3]].copy()
        des_velocities[: ,[4,5,6]], des_velocities[: ,[7,8,9]] = des_velocities[: ,[7,8,9]], des_velocities[:, [4,5,6]].copy()

        des_lqr_gains[:, 0:27], des_lqr_gains[:, 81:108 ] = des_lqr_gains[: ,81:108], des_lqr_gains[:, 0:27].copy()
        des_lqr_gains[:, 27:54], des_lqr_gains[:, 54:81 ] = des_lqr_gains[: ,54:81], des_lqr_gains[:, 27:54].copy()


        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))


        ## Converting des_pos and des_vel of end effector to a 6d vector per leg
        ## for the impedance controller
        ## so in total n*24
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))

                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))

        ## spliting into three parts because sot reader can load upto 40 columns only
        des_lqr_gains = np.multiply(-1, des_lqr_gains)
        des_lqr1 = des_lqr_gains[: ,0:36]
        des_lqr2 = des_lqr_gains[: ,36:72]
        des_lqr3 = des_lqr_gains[: ,72:108]
        #print(des_lqr3[0])
        # print(np.shape(des_lqr1))

        #print(desired_pos)
        print("saving trajectories....")
        np.savetxt("quadruped_positions_eff.dat", des_positions_final)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities_final)
        np.savetxt("quadruped_com.dat", des_com)
        np.savetxt("quadruped_lmom.dat", des_lmom)
        np.savetxt("quadruped_amom.dat", des_amom)
        np.savetxt("quadruped_lqr1.dat", des_lqr1)
        np.savetxt("quadruped_lqr2.dat", des_lqr2)
        np.savetxt("quadruped_lqr3.dat", des_lqr3)
        # np.savetxt("quadruped_lqr.dat", des_lqr_gains)
        np.savetxt("quadruped_forces.dat", des_forces)
        np.savetxt("quadruped_centroidal_forces.dat", des_centroidal_forces)
        np.savetxt("quadruped_centroidal_moments.dat", des_centroidal_moments)
        np.savetxt("quadruped_quaternion.dat", des_quaternion)
