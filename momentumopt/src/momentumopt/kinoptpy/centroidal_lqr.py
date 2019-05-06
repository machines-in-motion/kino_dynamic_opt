### computes gains using lqr in the centroidal space for solo (assumes legs are weightless)
### Author: Avadesh meduri
### Date:6/5/2019


import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as Rot


class centroidal_lqr:

    def __init__(self, dir):

        self.com_pos = np.loadtxt(dir + "/quadruped_com.dat", dtype=float)[:, [1,2,3]]
        self.com_vel = np.loadtxt(dir + "/quadruped_com_vel.dat", dtype=float)[:, [1,2,3]]
        self.com_ori = np.loadtxt(dir + "/quadruped_quaternion.dat", dtype=float)[:, [1,2,3,4]]
        self.com_ang_vel = np.loadtxt(dir + "/quadruped_base_ang_velocities.dat", dtype=float)[:, [1,2,3]]

        self.cent_force = np.loadtxt(dir + "/quadruped_centroidal_forces.dat", dtype=float)[:, [1,2,3]]
        self.cent_moments = np.loadtxt(dir + "/quadruped_centroidal_moments.dat", dtype=float)[:, [1,2,3]]

        self.dt = 0.001
        self.mass = 2.17
        self.inertia_com_frame = [[0.00578574, 0.0, 0.0],
                                  [0.0, 0.01938108, 0.0],
                                  [0.0, 0.0, 0.02476124]]


    def compute_dyn(self,t , x_t, u_t):

        ### quat_d = omega * quat

        self.omega = np.array([[0, self.com_ang_vel[t][2], -1*self.com_ang_vel[t][1], self.com_ang_vel[t][0]],
                      [-1*self.com_ang_vel[t][2], 0, self.com_ang_vel[t][0], self.com_ang_vel[t][1]],
                      [self.com_ang_vel[t][1], -1*self.com_ang_vel[t][0], 0, self.com_ang_vel[t][2]],
                      [-1*self.com_ang_vel[t][0], -1*self.com_ang_vel[t][1], -1*self.com_ang_vel[t][2], 0]])

        self.A_t = np.block([[np.zeros((3,3)), np.identity(3), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))],
                    [np.zeros((4,3)),np.zeros((4,3)), self.omega, np.zeros((4,3))],
                    [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3))]])


        ## could be wrong (quat sequence and the output)
        rot_t = Rot.from_quat(self.com_ori[t]).as_dcm()
        self.inertia = np.matmul(np.matmul(np.transpose(rot_t),self.inertia_com_frame), rot_t)
        self.inv_inertia = inv(np.matrix(self.inertia))



        self.B_t = np.block([[np.zeros((3,3)), np.zeros((3,3))],
                    [(1/self.mass)*np.identity(3), np.zeros((3,3))],
                    [np.zeros((4,3)), np.zeros((4,3))],
                    [np.zeros((3,3)), self.inv_inertia]])


        self.A_t = np.matrix(self.A_t)
        self.B_t = np.matrix(self.B_t)


        return np.matmul(self.A_t, np.transpose(x_t)) + np.matmul(self.B_t, np.transpose(u_t))

    def compute_lin_dyn(self,t):

        ### computes linearized dymamics

        x_t = np.matrix(np.hstack((self.com_pos[t], self.com_vel[t], self.com_ori[t], self.com_ang_vel[t])))
        u_t = np.matrix(np.hstack((self.cent_force[t], self.cent_moments[t])))

        dyn_t = self.compute_dyn(t, x_t, u_t)

        # print(dyn_t)

        # partial derivative of a w.r.t x
        x_t1 = np.matrix(np.hstack((self.com_pos[t+1], self.com_vel[t+1], self.com_ori[t+1], self.com_ang_vel[t+1])))
        u_t1 = np.matrix(np.hstack((self.cent_force[t+1], self.cent_moments[t+1])))

        print(x_t1 - x_t)

        lin_A_t = np.zeros((13,13))

        for i in range(13):
            print(i)
            pd_x_t = x_t
            delta_x = x_t1[: ,i] - x_t[: ,i]
            print(delta_x)
            pd_x_t[: ,i] = x_t1[: ,i]
            lin_A_t[:, i] = np.reshape(((self.compute_dyn(t, pd_x_t, u_t) - dyn_t.copy())/(delta_x)), (13,))
            # print(lin_A_t[:, i])
        # print("\n")


        lin_B_t = np.zeros((13,6))

        for i in range(6):
            pd_u_t = u_t
            delta_u = u_t1[: ,i] - u_t[:, i]
            pd_u_t[: ,i] = u_t1[:, i]
            lin_B_t[:, i] = np.reshape(((self.compute_dyn(t, x_t, pd_u_t) - dyn_t)/(delta_u)), (13,))

        return lin_A_t, lin_B_t

    def compute_lqr_gains(self, Q, R, lin_A_t, lin_B_t, P_prev):

        ## solves ricati equation
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

        for t in range(horizon-2, 0, -1):
            # print(t)
            lin_A_t, lin_B_t = self.compute_lin_dyn(t)
            K_t, P_prev = self.compute_lqr_gains(Q, R, lin_A_t, lin_B_t, P_prev)
            K_array.append(K_t)

            # print(K_t)
            print("\n")

        return np.asarray(K_array)


#### test
tmp = centroidal_lqr("../../../../momentumopt/demos")

# lin_A, lin_B = tmp.compute_lin_dyn(0)
# K, P = tmp.compute_lqr_gains(np.identity(13), np.identity(6), lin_A, lin_B, np.zeros((13,13)))

Q = 1000000*np.identity(13)
R = 10*np.identity(6)

K_array = tmp.lqr_backward_pass(Q,R)
