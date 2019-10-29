'''
@file test_quadruped_problems.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08

here we test all the functionalities of lqr_gain__manifold
'''

from utils import assert_all_close
from momentumopt.kino_dyn_planner_solo import build_and_optimize_motion
from momentumopt.quadruped.quadruped_wrapper import QuadrupedWrapper
import numpy as np
import unittest
import tempfile
from os import path, chdir, getcwd, walk
import eigenpy
eigenpy.switchToNumpyMatrix()


class CD:
    """Context manager for changing the current working directory"""

    def __init__(self, newPath):
        self.newPath = path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = getcwd()
        chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        chdir(self.savedPath)


class TestRobotJumps(unittest.TestCase):

    def setUp(self):
        super(TestRobotJumps, self).setUp()

    def tearDown(self):
        super(TestRobotJumps, self).tearDown()

    def optimize_the_jumping_motion(self, config_file_name, with_lqr):
        """ Build the optmization problem, optimize it and dump the data"""

        # get the config file
        yaml_file = path.join(self.yaml_config_dir, config_file_name)

        # Build the optmization problem
        motion_planner, optimized_kin_plan, optimized_motion_eff, \
            optimized_dyn_plan, dynamics_feedback, planner_setting, \
            time_vector = build_and_optimize_motion(
                yaml_file, QuadrupedWrapper,
                with_lqr=True, plot_com_motion=False)

        # Save the data files in a temporary folder
        with CD(self.data_dir):
            motion_planner.save_qp_files()

        self.motion_planner = motion_planner
        self.optimized_kin_plan = optimized_kin_plan
        self.optimized_motion_eff = optimized_motion_eff
        self.optimized_dyn_plan = optimized_dyn_plan
        self.dynamics_feedback = dynamics_feedback
        self.planner_setting = planner_setting
        self.time_vector = time_vector

    def check_generated_data(self, old_data_dir):
        old_data = self.load_data_from_files(old_data_dir)
        new_data = self.load_data_from_files(self.data_dir)

        self.assertEqual(old_data.keys(), new_data.keys())

        for key in old_data:
            assert_all_close(old_data[key], new_data[key], 1e-05)

    def load_data_from_files(self, folder_name):
        data = dict()
        for (dirpath, _, filenames) in walk(folder_name):
            for filename in filenames:
                key, _ = path.splitext(path.basename(filename))
                data[key] = np.loadtxt(path.join(dirpath, filename))
        return data

    def test_cfg_solo_jump_with_lqr(self):
        # Setup
        pkg_path = path.dirname(path.abspath(path.curdir))
        yaml_config_dir = path.join(pkg_path, "config")
        precomputed_data_folder = path.join(
            pkg_path, "tests", "jump_trajectories", "solo_jump_with_lqr")
        # create an empty directory to store the data
        self.data_dir = tempfile.mkdtemp()
        # Test the setup
        self.assertTrue(path.exists(yaml_config_dir))
        self.assertTrue(path.exists(precomputed_data_folder))

        # compute the motion for the jump
        self.optimize_the_jumping_motion(
            config_file_name="cfg_solo_jump.yaml", with_lqr=True)

        # load and check the data
        self.check_generated_data(precomputed_data_folder)

    def test_cfg_solo_jump_without_lqr(self):
        # Setup
        pkg_path = path.dirname(path.abspath(path.curdir))
        yaml_config_dir = path.join(pkg_path, "config")
        precomputed_data_folder = path.join(
            pkg_path, "tests", "jump_trajectories",
            "solo_jump_without_lqr")
        # create an empty directory to store the data
        self.data_dir = tempfile.mkdtemp()
        # Test the setup
        self.assertTrue(path.exists(yaml_config_dir))
        self.assertTrue(path.exists(precomputed_data_folder))

        # compute the motion for the jump
        self.optimize_the_jumping_motion(
            config_file_name="cfg_solo_jump.yaml", with_lqr=True)

        # load and check the data
        self.check_generated_data(precomputed_data_folder)
