""" here we test all the functionalities of lqr_gain__manifold """

# TODO: test dynamics integration if it matches with trajectory generated from planner
# TODO: test dynamics derivatives derivatives
# TODO: test cost function
# TODO: test cost derivatives
# TODO: test initialization and dimensions


import unittest
import numpy as np
import lqr_gain_manifold
import pinocchio as se3
import matplotlib.pyplot as plt
class TestDifferentialDynamicProgramming(unittest.TestCase):
    def test_initialization(self):
        """ check dimensions upon initialization """
        lqr_solver = lqr_gain_manifold.CentroidalLqr("../../../../momentumopt/demos")
        assert lqr_solver.com_pos.shape[1]+lqr_solver.com_vel.shape[1]\
            +lqr_solver.com_ori.shape[1]+lqr_solver.com_ang_vel.shape[1] == lqr_solver.n
        assert lqr_solver.com_pos.shape[0] == lqr_solver.N
        assert lqr_solver.com_vel.shape[0] == lqr_solver.N
        assert lqr_solver.com_ori.shape[0] == lqr_solver.N
        assert lqr_solver.com_ang_vel.shape[0] == lqr_solver.N
        assert lqr_solver.cent_force.shape[0] == lqr_solver.N
        assert lqr_solver.cent_moments.shape[0] == lqr_solver.N
        assert lqr_solver.cent_force.shape[1] + \
            lqr_solver.cent_moments.shape[1] == lqr_solver.m

    # def test_quaternion_to_rotation(self):
    #     '''check if rotation matrix from quaternion satisfies all conditions '''
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     for t in range(lqr_solver.N):
    #         rotation = lqr_solver.quaternion_to_rotation(lqr_solver.com_ori[t])
    #         np.testing.assert_array_almost_equal(rotation.dot(rotation.T), np.eye(3))
    #         np.testing.assert_almost_equal(np.linalg.det(rotation), 1.)

    # def test_quaternion_integration(self):
    #     """ test if integrating quaternion with angular
    #     velocity matches obtained quaternion trajectory """
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     quat_trajectory = np.zeros((lqr_solver.N,4))
    #     quat_trajectory[0] = lqr_solver.com_ori[0].copy()
    #     for t in range(lqr_solver.N-1):
    #         quat_trajectory[t+1] = lqr_solver.integrate_quaternion(quat_trajectory[t],
    #                                lqr_solver.com_ang_vel[t])
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['x', 'y', 'z', 'w']
    #     for i in range(4):
    #         plt.figure('quat_'+names[i])
    #         plt.plot(time_array, lqr_solver.com_ori[:, i], label='solver')
    #         plt.plot(time_array, quat_trajectory[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title('quat_'+names[i])
    #     plt.show()
    #     np.testing.assert_array_almost_equal(quat_trajectory,
    #                              lqr_solver.com_ori, decimal=4)

    # def test_position_integration(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = lqr_solver.x0.copy()
    #     pos = np.zeros((lqr_solver.N, 3))
    #     pos[0] =x[0,:3].copy()
    #     for t in range(lqr_solver.N-1):
    #         pos[t+1] = lqr_solver.integrate_position(pos[t], x[t,3:6], x[t, 6:10])
    #
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['cx', 'cy', 'cz']
    #     for i,n in enumerate(names):
    #         plt.figure(n)
    #         plt.plot(time_array, lqr_solver.x0[:, i], label='solver')
    #         plt.plot(time_array, pos[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title(n)
    #
    #     plt.show()


    # def test_velocity_integration(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = lqr_solver.x0.copy()
    #     vel = np.zeros((lqr_solver.N, 3))
    #     vel[0] = x[0, 3:6].copy()
    #     for t in range(lqr_solver.N-1):
    #         vel[t+1] = lqr_solver.integrate_veocity(vel[t], lqr_solver.cent_force[t])
    #
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['vx', 'vy', 'vz']
    #     for i, n in enumerate(names):
    #         plt.figure(n)
    #         plt.plot(time_array, lqr_solver.x0[:, i+3], label='solver')
    #         plt.plot(time_array, vel[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title(n)
    #     plt.show()

    # def test_angular_velocity__integration(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = lqr_solver.x0.copy()
    #     w = np.zeros((lqr_solver.N, 3))
    #     w[0] = x[0, 10:].copy()
    #     for t in range(lqr_solver.N-1):
    #         w[t+1] = lqr_solver.integrate_angular_velocity(w[t], x[t,6:10], lqr_solver.cent_moments[t])
    #
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['wx', 'wy', 'wz']
    #     for i, n in enumerate(names):
    #         plt.figure(n)
    #         plt.plot(time_array, lqr_solver.x0[:, i+10], label='solver')
    #         plt.plot(time_array, w[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title(n)
    #     plt.show()

    def test_integrate_step(self):
        lqr_solver = lqr_gain_manifold.CentroidalLqr(
            "../../../../momentumopt/demos")
        x = np.zeros((lqr_solver.N, lqr_solver.n))
        x[0] = lqr_solver.x0[0].copy()
        u = np.zeros((lqr_solver.N, lqr_solver.m))
        u[:, :3] = lqr_solver.cent_force.copy()
        u[:, 3:] = lqr_solver.cent_moments.copy()
        for t in range(lqr_solver.N-1):
            x[t+1] = lqr_solver.integrate_step(t, x[t], u[t])
        time_array = lqr_solver.dt * np.arange(lqr_solver.N)
        names = ['cx', 'cy', 'cz', 'vx', 'vy',
                 'vz', 'quatx', 'quaty', 'quatz', 'quatw', 'wx', 'wy', 'wz']

        for i in range(lqr_solver.n):
            plt.figure(names[i])
            plt.plot(time_array, lqr_solver.x0[:, i], label='solver')
            plt.plot(time_array, x[:, i], label='integrated')
            plt.grid()
            plt.legend()
            plt.title(names[i])
        plt.show()

if __name__ == '__main__':
    unittest.main()
