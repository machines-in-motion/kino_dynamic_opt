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

def create_qp_files(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    # desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    # desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_base_ang_velocity = interpolate("BASE_ANGULAR_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_vel_abs = interpolate("VELOCITY_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
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
        # des_amom = np.zeros((num_points, 4))
        # des_lqr_gains = np.zeros((num_points, 108))
        des_forces = np.zeros((num_points, 13))
        des_centroidal_forces = np.zeros((num_points, 4))
        des_centroidal_moments = np.zeros((num_points, 4))
        des_quaternion = np.zeros((num_points, 5))
        des_base_ang_velocities = np.zeros((num_points, 4))
        des_velocities_abs = np.zeros((num_points, 13))
        des_contact_activation = np.zeros((num_points, 5))


        for i in range(num_points):
            ## making des_pos and des_vel a 6d vector
            ### re arranging the sequence of legs to the latest from (FR, FL, HR, HL)
            ### to (FL, FR, HL, HR) for des_pos, des_vel and forces
            des_forces[i:, ] = np.hstack((i, desired_forces(i /1e3)))
            des_centroidal_forces[i:, ] = np.hstack((i, desired_centroidal_forces(i /1e3)))
            des_centroidal_moments[i:, ] = np.hstack((i, desired_centroidal_moments(i /1e3)))
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_com[i, :] = np.hstack((i, desired_com(i / 1e3)))
            des_lmom[i, :] = np.hstack((i, desired_lmom(i / 1e3)))
            # des_amom[i, :] = np.hstack((i, desired_amom(i / 1e3)))
            # des_lqr_gains_tmp = desired_lqr_gains(i / 1e3)
            # ## Converting lqr matrix (12 * 9) to a 108d vector
            # des_lqr_gains_tmp = np.reshape(des_lqr_gains_tmp, (108,))
            # des_lqr_gains[i,: ] = des_lqr_gains_tmp
            des_quaternion[i, :] = np.hstack((i, desired_quaternion(i / 1e3)))
            des_base_ang_velocities[i, :] = np.hstack((i, desired_base_ang_velocity(i / 1e3)))
            des_velocities_abs[i, :] = np.hstack((i, desired_vel_abs(i /1e3)))

        ## resequencing the eff sequence

        des_forces[: ,[1,2,3]], des_forces[: ,[4,5,6]] = des_forces[: ,[4,5,6]], des_forces[:, [1,2,3]].copy()
        des_forces[: ,[7,8,9]], des_forces[: ,[10,11,12]] = des_forces[: ,[10,11,12]], des_forces[:, [7,8,9]].copy()

        des_positions[: ,[1,2,3]], des_positions[: ,[4,5,6]] = des_positions[: ,[4,5,6]], des_positions[:, [1,2,3]].copy()
        des_positions[: ,[7,8,9]], des_positions[: ,[10,11,12]] = des_positions[: ,[10,11,12]], des_positions[:, [7,8,9]].copy()

        des_velocities[: ,[1,2,3]], des_velocities[: ,[4,5,6]] = des_velocities[: ,[4,5,6]], des_velocities[:, [1,2,3]].copy()
        des_velocities[: ,[7,8,9]], des_velocities[: ,[10,11,12]] = des_velocities[: ,[10,11,12]], des_velocities[:, [7,8,9]].copy()

        des_velocities_abs[: ,[1,2,3]], des_velocities_abs[: ,[4,5,6]] = des_velocities_abs[: ,[4,5,6]], des_velocities_abs[:, [1,2,3]].copy()
        des_velocities_abs[: ,[7,8,9]], des_velocities_abs[: ,[10,11,12]] = des_velocities_abs[: ,[10,11,12]], des_velocities_abs[:, [7,8,9]].copy()

        # filling contact switch vector in the horizon
        for i in range (num_points):
            des_contact_activation[i, 0]=i
            for j in range(4):
                if des_forces[i, 3*j+3]>0:
                    des_contact_activation[i, j+1]=1
                else:
                    des_contact_activation[i, j+1]=0

        # des_lqr_gains[:, 0:27], des_lqr_gains[:, 81:108 ] = des_lqr_gains[: , 81:108], des_lqr_gains[:, 0:27].copy()
        # des_lqr_gains[:, 27:54], des_lqr_gains[:, 54:81 ] = des_lqr_gains[: , 54:81], des_lqr_gains[:, 27:54].copy()
        #
        # des_lqr_gains[:, 0:27], des_lqr_gains[:, 27:54 ] = des_lqr_gains[: , 27:54], des_lqr_gains[:, 0:27].copy()
        # des_lqr_gains[:, 54:81], des_lqr_gains[:, 81:108 ] = des_lqr_gains[: , 81:108], des_lqr_gains[:, 54:81].copy()


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
        # des_lqr_gains = np.multiply(-1, des_lqr_gains)
        # des_lqr1 = des_lqr_gains[: ,0:36]
        # des_lqr2 = des_lqr_gains[: ,36:72]
        # des_lqr3 = des_lqr_gains[: ,72:108]
        #print(des_lqr3[0])
        # print(np.shape(des_lqr1))


        print("saving trajectories....")
        np.savetxt("quadruped_positions_eff.dat", des_positions_final)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities_final)
        np.savetxt("quadruped_com.dat", des_com)
        np.savetxt("quadruped_com_vel.dat", des_lmom/(robot_weight/9.81))
        np.savetxt("quadruped_centroidal_forces.dat", des_centroidal_forces)
        np.savetxt("quadruped_centroidal_moments.dat", des_centroidal_moments)
        np.savetxt("quadruped_quaternion.dat", des_quaternion)
        np.savetxt("quadruped_base_ang_velocities.dat", des_base_ang_velocities)
        np.savetxt("quadruped_velocities_abs.dat", des_velocities_abs)
        np.savetxt("quadruped_contact_activation.dat", des_contact_activation)



def create_lqr_files(time_vector, optimized_motion_eff, optimized_sequence, optimized_dyn_plan, dynamics_feedback, robot_weight):
    desired_pos = interpolate("POSITION", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_vel = interpolate("VELOCITY", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_com = interpolate("COM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_lmom = interpolate("LMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_amom = interpolate("AMOM", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)
    desired_forces = interpolate("FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, dynamics_feedback = dynamics_feedback)
    desired_lqr_gains = interpolate("DYN_FEEDBACK", time_vector, dynamics_feedback = dynamics_feedback)
    desired_quaternion = interpolate("QUATERNION", time_vector, optimized_sequence=optimized_sequence)
    desired_centroidal_forces = interpolate("CENTROIDAL_FORCES", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_centroidal_moments = interpolate("CENTROIDAL_MOMENTS", time_vector, optimized_dyn_plan = optimized_dyn_plan, robot_weight = robot_weight)
    desired_base_ang_velocity = interpolate("BASE_ANGULAR_VELOCITY", time_vector, optimized_sequence=optimized_sequence)
    desired_pos_abs = interpolate("POSITION_ABSOLUTE", time_vector, optimized_motion_eff=optimized_motion_eff, optimized_sequence = optimized_sequence)

    max_time = 0 # time horizon in seconds
    save_horizon = 4

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
        des_forces = np.zeros((num_points, 12*(save_horizon+1)+1))
        des_com = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_lmom = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_quaternion = np.zeros((num_points, 4*(save_horizon+1)+1))
        des_base_ang_velocities = np.zeros((num_points, 3*(save_horizon+1)+1))
        des_positions_abs = np.zeros((num_points, 12*(save_horizon+1)+1))
        des_contact_activation = np.zeros((num_points, 4*(save_horizon+1)+1))

        for i in range(num_points):
            des_positions[i, :] = np.hstack((i, desired_pos(i / 1e3)))
            des_velocities[i, :] = np.hstack((i, desired_vel(i / 1e3)))
            des_forces[i, 0:13] = np.hstack((i, desired_forces(i /1e3)))
            des_com[i, 0:4] = np.hstack((i, desired_com(i / 1e3)))
            des_lmom[i, 0:4] = np.hstack((i, desired_lmom(i / 1e3)))
            des_quaternion[i, 0:5] = np.hstack((i, desired_quaternion(i / 1e3)))
            des_base_ang_velocities[i, 0:4] = np.hstack((i, desired_base_ang_velocity(i / 1e3)))
            des_positions_abs[i, 0:13] = np.hstack((i, desired_pos_abs(i /1e3)))
            for j in range(save_horizon):
                if i<num_points-save_horizon:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((i+j+1) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((i+j+1) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((i+j+1) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((i+j+1) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((i+j+1) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((i+j+1) / 1e3)
                else:
                    des_com[i, 3*(j+1)+1:3*(j+2)+1] = desired_com((num_points) / 1e3)
                    des_lmom[i, 3*(j+1)+1:3*(j+2)+1] = desired_lmom((num_points) / 1e3)
                    des_base_ang_velocities[i, 3*(j+1)+1:3*(j+2)+1] = desired_base_ang_velocity((num_points) / 1e3)
                    des_quaternion[i, 4*(j+1)+1:4*(j+2)+1] = desired_quaternion((num_points) / 1e3)
                    des_forces[i, 12*(j+1)+1:12*(j+2)+1] = desired_forces((num_points) / 1e3)
                    des_positions_abs[i, 12*(j+1)+1:12*(j+2)+1] = desired_pos_abs((num_points) / 1e3)

        ## resequencing the eff sequence
        # des_forces[: ,[1,2,3]], des_forces[: ,[10,11,12]] =\
        # des_forces[: ,[10,11,12]], des_forces[:, [1,2,3]].copy()
        # des_forces[: ,[4,5,6]], des_forces[: ,[7,8,9]] = \
        # des_forces[: ,[7,8,9]], des_forces[:, [4,5,6]].copy()
        for j in range(save_horizon+1):
            des_forces[: ,[12*j+1,12*j+2,12*j+3]], des_forces[: ,[12*j+10,12*j+11,12*j+12]] =\
            des_forces[: ,[12*j+10,12*j+11,12*j+12]], des_forces[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_forces[: ,[12*j+4,12*j+5,12*j+6]], des_forces[: ,[12*j+7,12*j+8,12*j+9]] = \
            des_forces[: ,[12*j+7,12*j+8,12*j+9]], des_forces[:, [12*j+4,12*j+5,12*j+6]].copy()

            des_positions_abs[: ,[12*j+1,12*j+2,12*j+3]], des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]] =\
            des_positions_abs[: ,[12*j+10,12*j+11,12*j+12]], des_positions_abs[:, [12*j+1,12*j+2,12*j+3]].copy()
            des_positions_abs[: ,[12*j+4,12*j+5,12*j+6]], des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]] = \
            des_positions_abs[: ,[12*j+7,12*j+8,12*j+9]], des_positions_abs[:, [12*j+4,12*j+5,12*j+6]].copy()

        # filling contact switch vector in the horizon
        for i in range (num_points):
            des_contact_activation[i, 0]=i
            for j in range(4*(save_horizon+1)):
                if des_forces[i, 3*j+3]>0:
                    des_contact_activation[i, j+1]=1
                else:
                    des_contact_activation[i, j+1]=0

        des_positions[: ,[1,2,3]], des_positions[: ,[4,5,6]] = des_positions[: ,[4,5,6]], des_positions[:, [1,2,3]].copy()
        des_positions[: ,[7,8,9]], des_positions[: ,[10,11,12]] = des_positions[: ,[10,11,12]], des_positions[:, [7,8,9]].copy()

        des_velocities[: ,[1,2,3]], des_velocities[: ,[4,5,6]] = des_velocities[: ,[4,5,6]], des_velocities[:, [1,2,3]].copy()
        des_velocities[: ,[7,8,9]], des_velocities[: ,[10,11,12]] = des_velocities[: ,[10,11,12]], des_velocities[:, [7,8,9]].copy()

        des_positions_final = np.zeros((num_points, 24))
        des_velocities_final = np.zeros((num_points, 24))


        ## Converting des_pos and des_vel of end effector to a 6d vector per leg
        ## for the impedance controller
        ## so in total n*24
        for i in range(num_points):
            for eff in range(4):
                des_positions_final[i][6*eff:6*(eff+1)] = np.hstack((des_positions[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))
                des_velocities_final[i][6*eff:6*(eff+1)] = np.hstack((des_velocities[i][3*(eff)+1:3*(eff+1) + 1], [0.0, 0.0, 0.0]))

        des_forces*=robot_weight
        #print(desired_pos)
        print("saving trajectories....")
        np.savetxt("quadruped_positions_eff.dat", des_positions_final)
        np.savetxt("quadruped_velocities_eff.dat", des_velocities_final)
        np.savetxt("quadruped_com_with_horizon.dat", des_com)
        np.savetxt("quadruped_lmom_with_horizon.dat", des_lmom)
        np.savetxt("quadruped_forces_with_horizon_part1.dat", des_forces[:,0:37])
        np.savetxt("quadruped_forces_with_horizon_part2.dat", des_forces[:,37:])
        np.savetxt("quadruped_quaternio_with_horizon.dat", des_quaternion)
        np.savetxt("quadruped_base_ang_velocities_with_horizon.dat", des_base_ang_velocities)
        np.savetxt("quadruped_positions_abs_with_horizon_part1.dat", des_positions_abs[:,0:37])
        np.savetxt("quadruped_positions_abs_with_horizon_part2.dat", des_positions_abs[:,37:])
        np.savetxt("des_contact_activation_with_horizon.dat", des_contact_activation)
