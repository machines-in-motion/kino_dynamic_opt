'''
@file test_lqr.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08

here we test all the functionalities of lqr_gain__manifold
'''

# TODO: test dynamics integration if it matches with trajectory generated from
#       planner
# TODO: test dynamics derivatives derivatives
# TODO: test cost function
# TODO: test cost derivatives
# TODO: test initialization and dimensions

from momentumopt.kinoptpy import lqr_gain_manifold
from shutil import copy
import unittest
import tempfile
import os
from os import path, walk
import numpy as np
import eigenpy
import pytest


class CD:
    """Context manager for changing the current working directory"""

    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)


class TestDifferentialDynamicProgramming(unittest.TestCase):
    """Test class for the lqr on a specific test case (quadruped jump)"""

    def setUp(self):
        # get the path this package
        self.pkg_path = path.dirname(path.abspath(path.dirname(__file__)))
        # get the path to the reference trajectories
        self.data_ref_dir = path.join(self.pkg_path, "tests",
                                      "jump_trajectories",
                                      "solo_jump_with_lqr")
        # check that the reference path exists
        self.assertTrue(path.exists(self.data_ref_dir))
        # create a temporary empty directory
        self.data_dir = tempfile.mkdtemp()
        # copy the reference trajectory inside
        for (dirpath, _, filenames) in walk(self.data_ref_dir):
            for filename in filenames:
                copy(path.join(dirpath, filename), self.data_dir)

        super(TestDifferentialDynamicProgramming, self).setUp()

    def tearDown(self):
        super(TestDifferentialDynamicProgramming, self).tearDown()

    def test_initialization(self):
        """ check dimensions upon initialization """
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        assert (lqr_solver.com_pos.shape[1] + lqr_solver.com_vel.shape[1] +
                lqr_solver.com_ori.shape[1] + lqr_solver.com_ang_vel.shape[1]
                == lqr_solver.n)
        assert lqr_solver.com_pos.shape[0] == lqr_solver.N+1
        assert lqr_solver.com_vel.shape[0] == lqr_solver.N+1
        assert lqr_solver.com_ori.shape[0] == lqr_solver.N+1
        assert lqr_solver.com_ang_vel.shape[0] == lqr_solver.N+1
        assert lqr_solver.cent_force.shape[0] == lqr_solver.N+1
        assert lqr_solver.cent_moments.shape[0] == lqr_solver.N+1
        assert lqr_solver.cent_force.shape[1] + \
            lqr_solver.cent_moments.shape[1] == lqr_solver.m

    def test_quaternion_to_rotation(self):
        """check if rotation matrix from quaternion satisfies all group
        conditions"""
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        for t in range(lqr_solver.N+1):
            rotation = lqr_solver.quaternion_to_rotation(lqr_solver.com_ori[t])
            np.testing.assert_array_almost_equal(rotation.dot(rotation.T),
                                                 np.eye(3))
            np.testing.assert_almost_equal(np.linalg.det(rotation), 1.)

    def test_quaternion_integration(self):
        """ test if integrating quaternion with angular
        velocity matches obtained quaternion trajectory """
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        quat_trajectory = np.zeros((lqr_solver.N+1, 4))
        quat_trajectory[0] = lqr_solver.com_ori[0].copy()
        for t in range(lqr_solver.N):
            quat_trajectory[t+1] = lqr_solver.integrate_quaternion(
                quat_trajectory[t], lqr_solver.dt * lqr_solver.com_ang_vel[t])
        # time_array = lqr_solver.dt * np.arange(lqr_solver.N)
        # names = ['x', 'y', 'z', 'w']
        # for i in range(4):
        #     plt.figure('quat_'+names[i])
        #     plt.plot(time_array, lqr_solver.com_ori[:, i], label='solver')
        #     plt.plot(time_array, quat_trajectory[:, i], label='integrated')
        #     plt.grid()
        #     plt.legend()
        #     plt.title('quat_'+names[i])
        # plt.show()
        np.testing.assert_array_almost_equal(quat_trajectory,
                                             lqr_solver.com_ori, decimal=4)

    def test_explog_quaternion(self):
        """ test if q = exp(log(quat)) """
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        quat_trajectory = lqr_solver.com_ori.copy()
        for i in range(quat_trajectory.shape[0]):
            qnew = lqr_solver.exp_quaternion(.5 * lqr_solver.log_quaternion(
                quat_trajectory[i]))
            np.testing.assert_array_almost_equal(quat_trajectory[i], qnew)

    def test_quaternion_difference(self):
        ''' check if computed quaternion differences equal scaled angular
        velocities '''
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        quat_trajectory = lqr_solver.com_ori.copy()
        for i in range(10):
            w = np.random.rand(3)
            q2 = lqr_solver.integrate_quaternion(quat_trajectory[i], w)
            diff = lqr_solver.quaternion_difference(quat_trajectory[i], q2)
            # integrate quaternion accounts for dt, hence need to divide by it
            np.testing.assert_almost_equal(w, diff)

    def test_dynamics_derivatives(self):
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)

        fx, fu = lqr_solver.dynamics_derivatives(0, lqr_solver.x0[0],
                                                 lqr_solver.u0[0])
        # can test smaller blocks individually, will do that later but seems
        # ok for now

        # print (fx)
        # print ('===================')
        # print (fu)

    def test_cost_along_trajectory(self):
        '''tests computation of cost residuals along a reference trajectory '''
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        c = 0.0
        # here we compute cost of planner trajectory compared to integrated
        # trajectory
        for t in range(lqr_solver.N):
            if t == lqr_solver.N - 1:
                c += lqr_solver.cost(t,
                                     lqr_solver.xp[t], np.zeros(lqr_solver.m))
            else:
                c += lqr_solver.cost(t, lqr_solver.xp[t], lqr_solver.u0[t])

        print ('cost along trajectory = ', c)

    def test_cost_derivatives(self):
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)

        cx, cu, cxx, cuu, cxu = lqr_solver.cost_derivatives(
            0, lqr_solver.x0[0], lqr_solver.u0[0])
        # print (cx)
        # print ('===================')
        # print (cu)
        # print ('===================')
        # print (cxx)
        # # print ('===================')
        # # print (cuu)
        # # print ('===================')
        # # print (cxu)

    @pytest.mark.skip(reason="This test is disabled as it is too slow")
    def test_compute_gains(self):
        """ This test is disabled as it is too slow """
        lqr_solver = lqr_gain_manifold.CentroidalLqr(self.data_dir)
        lqr_solver.compute_gains()

        feedback = lqr_solver.kfb.copy()
        time = lqr_solver.dt * np.arange(feedback.shape[0])
        norms = []
        for t in range(feedback.shape[0]):
            norms += [np.linalg.norm(feedback[t])]

        # plt.figure('gains')
        # plt.plot(time, norms)
        # plt.grid()
        # plt.show()

    # def test_position_integration(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = lqr_solver.x0.copy()
    #     pos = np.zeros((lqr_so        # time_array = lqr_solver.dt * np.arange(lqr_solver.N)
        # names = ['x', 'y', 'z', 'w']
        # for i in range(4):
        #     plt.figure('quat_'+names[i])
        #     plt.plot(time_array, lqr_solver.com_ori[:, i], label='solver')
        #     plt.plot(time_array, quat_trajectory[:, i], label='integrated')
        #     plt.grid()
        #     plt.legend()
        #     plt.title('quat_'+names[i])
        # plt.show()
    #     pos[0] =x[0,:3].copy()        # time_array = lqr_solver.dt * np.arange(lqr_solver.N)
        # names = ['x', 'y', 'z', 'w']
        # for i in range(4):
        #     plt.figure('quat_'+names[i])
        #     plt.plot(time_array, lqr_solver.com_ori[:, i], label='solver')
        #     plt.plot(time_array, quat_trajectory[:, i], label='integrated')
        #     plt.grid()
        #     plt.legend()
        #     plt.title('quat_'+names[i])
        # plt.show()
    #     for t in range(lqr_solver.N-1):
    #         pos[t+1] = lqr_solver.integrate_position(pos[t], x[t,3:6], x[t, 6:10])

    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['cx', 'cy', 'cz']
    #     for i,n in enumerate(names):
    #         plt.figure(n)
    #         plt.plot(time_array, lqr_solver.x0[:, i], label='solver')
    #         plt.plot(time_array, pos[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title(n)

    #     plt.show()

    # def test_velocity_integration(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = lqr_solver.x0.copy()
    #     vel = np.zeros((lqr_solver.N, 3))
    #     vel[0] = x[0, 3:6].copy()
    #     for t in range(lqr_solver.N-1):
    #         vel[t+1] = lqr_solver.integrate_veocity(vel[t], lqr_solver.cent_force[t])

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

    # def test_integrate_step(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")
    #     x = np.zeros((lqr_solver.N, lqr_solver.n))
    #     x[0] = lqr_solver.x0[0].copy()
    #     u = np.zeros((lqr_solver.N, lqr_solver.m))
    #     u[:, :3] = lqr_solver.cent_force.copy()
    #     u[:, 3:] = lqr_solver.cent_moments.copy()
    #     for t in range(lqr_solver.N-1):
    #         x[t+1] = lqr_solver.integrate_step(t, x[t], u[t])
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)
    #     names = ['cx', 'cy', 'cz', 'vx', 'vy',
    #              'vz', 'quatx', 'quaty', 'quatz', 'quatw', 'wx', 'wy', 'wz']

    #     for i in range(lqr_solver.n):
    #         plt.figure(names[i])
    #         plt.plot(time_array, lqr_solver.x0[:, i], label='solver')
    #         plt.plot(time_array, x[:, i], label='integrated')
    #         plt.grid()
    #         plt.legend()
    #         plt.title(names[i])
    #     plt.show()

    # def test_compute_trajectory_from_controls(self):
    #     lqr_solver = lqr_gain_manifold.CentroidalLqr(
    #         "../../../../momentumopt/demos")

    #     names = ['cx', 'cy', 'cz']
    #     names2 = ['vx', 'vy', 'vz']
    #     names3 = ['quatx', 'quaty', 'quatz', 'quatw']
    #     names4 = ['wx', 'wy', 'wz']
    #     time_array = lqr_solver.dt * np.arange(lqr_solver.N)

    #     plt.figure('CoM position')
    #     for i,n in enumerate(names):
    #         plt.plot(time_array, lqr_solver.x0[:,i], label=n)
    #     plt.grid()
    #     plt.legend()
    #     plt.title('CoM position')

    #     plt.figure('CoM velocity')
    #     for i,n in enumerate(names2):np.zeros((lqr_solver.N,4))
    #         plt.plot(time_array, lqr_np.zeros((lqr_solver.N,4))
    #     plt.grid()
    #     plt.legend()
    #     plt.title('CoM velocity')

    #     plt.figure('Base orientation')
    #     for i,n in enumerate(names3):
    #         plt.plot(time_array, lqr_solver.x0[:,i+6], label=n)
    #     plt.grid()
    #     plt.legend()
    #     plt.title('Base orientation')

    #     plt.figure('Base angular velocity')
    #     for i,n in enumerate(names4):
    #         plt.plot(time_array, lqr_solver.x0[:,i+10], label=n)
    #     plt.grid()
    #     plt.legend()
    #     plt.title('Base angular velocity')

    #     plt.show()


if __name__ == '__main__':
    unittest.main()
