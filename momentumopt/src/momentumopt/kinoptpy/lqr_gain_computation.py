### computes gains using lqr in the centroidal space for solo (assumes legs are weightless)
### computes gains using lqr in the end_effector space for solo (assumes legs are weightless)
### Performs a backward pass to compute gains using a trajectory
### Author: Avadesh meduri
### Date:6/5/2019


import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as Rot


np.set_printoptions(linewidth=13000)

class centroidal_lqr:

    def __init__(self, dir):

        self.dir = dir
        self.com_pos = np.loadtxt(dir + "/quadruped_com.dat", dtype=float)[:, [1,2,3]]
        self.com_vel = np.loadtxt(dir + "/quadruped_com_vel.dat", dtype=float)[:, [1,2,3]]
        self.com_ori = np.loadtxt(dir + "/quadruped_quaternion.dat", dtype=float)[:, [1,2,3,4]]
        self.com_ang_vel = np.loadtxt(dir + "/quadruped_base_ang_velocities.dat", dtype=float)[:, [1,2,3]]

        self.cent_force = np.loadtxt(dir + "/quadruped_centroidal_forces.dat", dtype=float)[:, [1,2,3]]
        self.cent_moments = np.loadtxt(dir + "/quadruped_centroidal_moments.dat", dtype=float)[:, [1,2,3]]

        self.delta = 0.000001
        self.dt = 0.001
        self.mass = 2.17
        self.inertia_com_frame = [[0.00578574, 0.0, 0.0],
                                  [0.0, 0.01938108, 0.0],
                                  [0.0, 0.0, 0.02476124]]


    def compute_dyn(self,t , x_t, u_t):

        ### quat_d = omega * quat
        omega = np.array([[0, x_t[: , 12], -1*x_t[:, 11], x_t[:, 10]],
                      [-1*x_t[:,12], 0, x_t[:,10], x_t[:, 11]],
                      [x_t[:,11], -1*x_t[:,10], 0, x_t[:,12]],
                      [-1*x_t[:, 10], -1*x_t[:, 11], -1*x_t[:,12], 0]])


        self.A_t = np.block([[np.zeros((3,3)), np.identity(3), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((4,3)),np.zeros((4,3)), 0.5*omega, np.zeros((4,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))]])

        rot_t = np.reshape(Rot.from_quat(x_t[:, [6,7,8,9]]).as_dcm(), (3,3))
        inertia = np.matmul(np.matmul(np.transpose(rot_t),self.inertia_com_frame), rot_t)
        inv_inertia = inv(np.matrix(inertia))



        self.B_t = np.block([[np.zeros((3,3)), np.zeros((3,3))],
                    [(1/self.mass)*np.identity(3), np.zeros((3,3))],
                    [np.zeros((4,3)), np.zeros((4,3))],
                    [np.zeros((3,3)), inv_inertia]])


        self.A_t = np.matrix(self.A_t)
        self.B_t = np.matrix(self.B_t)


        return np.matmul(self.A_t, np.transpose(x_t)) + np.matmul(self.B_t, np.transpose(u_t))

    def compute_lin_dyn(self,t):

        ### computes linearized dymamics

        x_t = np.matrix(np.hstack((self.com_pos[t], self.com_vel[t], self.com_ori[t], self.com_ang_vel[t])))
        u_t = np.matrix(np.hstack((self.cent_force[t], self.cent_moments[t])))

        dyn_t = self.compute_dyn(t, x_t, u_t)


        # partial derivative of a w.r.t x
        x_t1 = np.matrix(np.hstack((self.com_pos[t+1], self.com_vel[t+1], self.com_ori[t+1], self.com_ang_vel[t+1])))
        u_t1 = np.matrix(np.hstack((self.cent_force[t+1], self.cent_moments[t+1])))


        lin_A_t = np.zeros((13,13))

        for i in range(13):
            pd_x_t = x_t.copy()
            delta_x = x_t1[: ,i].copy() - x_t[: ,i].copy()
            pd_x_t[: ,i] = x_t1[: ,i].copy()
            if delta_x == 0.0:
                delta_x = self.delta
                pd_x_t[:, i] += self.delta
            lin_A_t[:, i] = np.reshape(((self.compute_dyn(t, pd_x_t, u_t) - dyn_t.copy())/(delta_x)), (13,))



        lin_B_t = np.zeros((13,6))
        if np.linalg.norm(sum(u_t1)) < 0.001:
            lin_B_t = np.zeros((13,6))
        else:
            for i in range(6):
                pd_u_t = u_t.copy()
                delta_u = u_t1[: ,i].copy() - u_t[:, i].copy()
                pd_u_t[: ,i] = u_t1[:, i].copy()
                if delta_u == 0:
                    delta_u = self.delta
                    pd_u_t[:, i] += self.delta
                lin_B_t[:, i] = np.reshape(((self.compute_dyn(t, x_t, pd_u_t) - dyn_t.copy())/(delta_u)), (13,))

        return lin_A_t, lin_B_t


    def descretise_dynamics(self, lin_A_t, lin_B_t):

        ## descritizes the dynamics adn returns descritized lin_A, lin_B_t

        des_lin_A_t = lin_A_t*self.dt + np.identity(13)
        des_lin_B_t = lin_B_t*self.dt

        # print(des_lin_A_t)
        return des_lin_A_t, des_lin_B_t


    def compute_lqr_gains(self, Q, R, lin_A_t, lin_B_t, P_prev):
        ## input descritzed lin_A and lin_B
        ## solves ricati equation
        # print(lin_B_t)
        K = inv(R + np.matmul(np.matmul(np.transpose(lin_B_t) , P_prev), lin_B_t))
        K = np.matmul(np.matmul(np.matmul(K, np.transpose(lin_B_t)), P_prev), lin_A_t)
        K = -1*K

        P = Q + np.matmul(np.matmul(np.transpose(K),R),K)
        P += np.matmul(np.matmul(np.transpose(lin_A_t + np.matmul(lin_B_t, K)), P_prev), lin_A_t + np.matmul(lin_B_t, K))

        return K, P


    def lqr_backward_pass(self, Q, R):

        horizon = len(self.com_pos)
        P_prev = np.zeros((13,13))
        K_array = []

        for t in range(horizon-2, -1, -1):
            print(t/1000.0)
            lin_A_t, lin_B_t = self.compute_lin_dyn(t)
            des_lin_A_t, des_lin_B_t = self.descretise_dynamics(lin_A_t, lin_B_t)
            K_t, P_prev = self.compute_lqr_gains(Q, R, des_lin_A_t, des_lin_B_t, P_prev)
            K_array.append(K_t)
            # print(P_prev)
            print(K_t)
            print("\n")
            # print("len", len(K_array))
            # print(horizon)

        return np.asarray(K_array)


    def store_lqr_gains(self, K_array):

        ## Stores gains as a 112d array
        K_array = np.reshape(K_array, (len(K_array), 78))

        np.savetxt(self.dir + "/quadruped_centroidal_gains1.dat", K_array[:,0:39])
        np.savetxt(self.dir + "/quadruped_centroidal_gains2.dat", K_array[:,39:])



class end_effector_lqr:

    def __init__(self, dir):

        self.dir = dir
        self.com_pos = np.loadtxt(dir + "/quadruped_com.dat", dtype=float)[:, [1,2,3]]
        self.com_vel = np.loadtxt(dir + "/quadruped_com_vel.dat", dtype=float)[:, [1,2,3]]
        self.com_ori = np.loadtxt(dir + "/quadruped_quaternion.dat", dtype=float)[:, [1,2,3,4]]
        self.com_ang_vel = np.loadtxt(dir + "/quadruped_base_ang_velocities.dat", dtype=float)[:, [1,2,3]]

        self.end_eff_forces = np.loadtxt(dir + "/quadruped_forces.dat", dtype=float)[:, 1:]
        self.end_eff_abs_pos = np.loadtxt(dir + "/quadruped_positions_abs_with_horizon_part.dat", dtype=float)[:, 1:]

        self.delta = 0.000001
        self.dt = 0.001
        self.mass = 2.17
        self.inertia_com_frame = [[0.00578574, 0.0, 0.0],
                                  [0.0, 0.01938108, 0.0],
                                  [0.0, 0.0, 0.02476124]]


    def compute_r_cross(self, end_eff_abs_pos, com_pos):

        r_cross_mat = [[0, -(end_eff_abs_pos[2] - com_pos[2]), (end_eff_abs_pos[1] - com_pos[1])],
                       [(end_eff_abs_pos[2] - com_pos[2]), 0, -(end_eff_abs_pos[0] - com_pos[0])],
                       [-(end_eff_abs_pos[1] - com_pos[1]), -(end_eff_abs_pos[0] - com_pos[0]), 0]]

        return r_cross_mat


    def compute_dyn(self,t , x_t, u_t):

        ### quat_d = omega * quat
        omega = np.array([[0, x_t[: , 12], -1*x_t[:, 11], x_t[:, 10]],
                      [-1*x_t[:,12], 0, x_t[:,10], x_t[:, 11]],
                      [x_t[:,11], -1*x_t[:,10], 0, x_t[:,12]],
                      [-1*x_t[:, 10], -1*x_t[:, 11], -1*x_t[:,12], 0]])


        self.A_t = np.block([[np.zeros((3,3)), np.identity(3), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((4,3)),np.zeros((4,3)), 0.5*omega, np.zeros((4,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))]])

        rot_t = np.reshape(Rot.from_quat(x_t[:, [6,7,8,9]]).as_dcm(), (3,3))
        inertia = np.matmul(np.matmul(np.transpose(rot_t),self.inertia_com_frame), rot_t)
        inv_inertia = inv(np.matrix(inertia))

        r_cross_inv_inertia_fl = np.matmul(inv_inertia, self.compute_r_cross(self.end_eff_abs_pos[t][0:3], self.com_pos[t]))
        r_cross_inv_inertia_fr = np.matmul(inv_inertia, self.compute_r_cross(self.end_eff_abs_pos[t][3:6], self.com_pos[t]))
        r_cross_inv_inertia_hl = np.matmul(inv_inertia, self.compute_r_cross(self.end_eff_abs_pos[t][6:9], self.com_pos[t]))
        r_cross_inv_inertia_hr = np.matmul(inv_inertia, self.compute_r_cross(self.end_eff_abs_pos[t][9:12], self.com_pos[t]))

        self.B_t = np.block([[np.zeros((3,3)), np.zeros((3,3)),np.zeros((3,3)), np.zeros((3,3))],
                            [(1/self.mass)*np.identity(3), (1/self.mass)*np.identity(3), (1/self.mass)*np.identity(3), (1/self.mass)*np.identity(3)],
                            [np.zeros((4,3)), np.zeros((4,3)), np.zeros((4,3)), np.zeros((4,3))],
                            [r_cross_inv_inertia_fl, r_cross_inv_inertia_fr, r_cross_inv_inertia_hl, r_cross_inv_inertia_hr]])


        self.A_t = np.matrix(self.A_t)
        self.B_t = np.matrix(self.B_t)

        return np.matmul(self.A_t, np.transpose(x_t)) + np.matmul(self.B_t, np.transpose(u_t))

    def compute_lin_dyn(self,t):

        ### computes linearized dymamics

        x_t = np.matrix(np.hstack((self.com_pos[t], self.com_vel[t], self.com_ori[t], self.com_ang_vel[t])))
        u_t = np.matrix(self.end_eff_forces[t])

        dyn_t = self.compute_dyn(t, x_t, u_t)
        # partial derivative of a w.r.t x
        x_t1 = np.matrix(np.hstack((self.com_pos[t+1], self.com_vel[t+1], self.com_ori[t+1], self.com_ang_vel[t+1])))
        u_t1 = np.matrix(self.end_eff_forces[t+1])


        lin_A_t = np.zeros((13,13))

        for i in range(13):
            pd_x_t = x_t.copy()
            delta_x = x_t1[: ,i].copy() - x_t[: ,i].copy()
            pd_x_t[: ,i] = x_t1[: ,i].copy()
            if delta_x == 0.0:
                delta_x = self.delta
                pd_x_t[:, i] += self.delta
            lin_A_t[:, i] = np.reshape(((self.compute_dyn(t, pd_x_t, u_t) - dyn_t.copy())/(delta_x)), (13,))



        lin_B_t = np.zeros((13,12))
        if np.linalg.norm(sum(u_t1)) < 0.001:
            lin_B_t = np.zeros((13,12))
        else:
            for i in range(12):
                pd_u_t = u_t.copy()
                delta_u = u_t1[: ,i].copy() - u_t[:, i].copy()
                pd_u_t[: ,i] = u_t1[:, i].copy()
                if delta_u == 0:
                    delta_u = self.delta
                    pd_u_t[:, i] += self.delta
                lin_B_t[:, i] = np.reshape(((self.compute_dyn(t, x_t, pd_u_t) - dyn_t.copy())/(delta_u)), (13,))

        return lin_A_t, lin_B_t


    def descretise_dynamics(self, lin_A_t, lin_B_t):

        ## descritizes the dynamics adn returns descritized lin_A, lin_B_t

        des_lin_A_t = lin_A_t*self.dt + np.identity(13)
        des_lin_B_t = lin_B_t*self.dt

        # print(des_lin_A_t)
        return des_lin_A_t, des_lin_B_t


    def compute_lqr_gains(self, Q, R, lin_A_t, lin_B_t, P_prev):
        ## input descritzed lin_A and lin_B
        ## solves ricati equation
        # print(lin_B_t)
        K = inv(R + np.matmul(np.matmul(np.transpose(lin_B_t) , P_prev), lin_B_t))
        K = np.matmul(np.matmul(np.matmul(K, np.transpose(lin_B_t)), P_prev), lin_A_t)
        K = -1*K

        P = Q + np.matmul(np.matmul(np.transpose(K),R),K)
        P += np.matmul(np.matmul(np.transpose(lin_A_t + np.matmul(lin_B_t, K)), P_prev), lin_A_t + np.matmul(lin_B_t, K))

        return K, P


    def lqr_backward_pass(self, Q, R):

        horizon = len(self.com_pos) - 100
        P_prev = np.zeros((13,13))
        K_array = []

        for t in range(horizon-2, -1, -1):
            print(t/1000.0)
            lin_A_t, lin_B_t = self.compute_lin_dyn(t)
            des_lin_A_t, des_lin_B_t = self.descretise_dynamics(lin_A_t, lin_B_t)
            K_t, P_prev = self.compute_lqr_gains(Q, R, des_lin_A_t, des_lin_B_t, P_prev)
            K_array.append(K_t)
            # print(P_prev)
            print(K_t)
            print("\n")
            # print("len", len(K_array))
            # print(horizon)

        return np.asarray(K_array)


    def store_lqr_gains(self, K_array):

        ## Stores gains as a 112d array
        K_array = np.reshape(K_array, (len(K_array), 78))

        np.savetxt(self.dir + "/quadruped_centroidal_gains1.dat", K_array[:,0:39])
        np.savetxt(self.dir + "/quadruped_centroidal_gains2.dat", K_array[:,39:])


#### test #####################################################################
Q = np.identity(13)
Q[0][0] = 1500
Q[1][1] = 1000
Q[2][2] = 1500
Q[3][3] = 0.01
Q[4][4] = 0.01
Q[5][5] = 0.01
Q[6][6] = 100
Q[7][7] = 100
Q[8][8] = 100
Q[9][9] = 100
Q[10][10] = 0.008
Q[11][11] = 0.008
Q[12][12] = 0.008

R = 0.1*np.identity(6)
R_eff = 0.1*np.identity(12)

# solo_cent_lqr_computer = centroidal_lqr("../../../../momentumopt/demos")
# K_array = solo_cent_lqr_computer.lqr_backward_pass(Q,R)
# solo_cent_lqr_computer.store_lqr_gains(K_array)

solo_end_eff_lqr = end_effector_lqr("../../../../momentumopt/demos")
K_array = solo_end_eff_lqr.lqr_backward_pass(Q,R_eff)
