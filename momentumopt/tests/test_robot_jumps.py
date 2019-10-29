'''
@file test_quadruped_problems.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08

here we test all the functionalities of lqr_gain__manifold
'''

import eigenpy
eigenpy.switchToNumpyMatrix()
import numpy as np
import unittest
import tempfile
from os import walk, path

from utils import assert_all_close, CD
from momentumopt.kino_dyn_planner_solo import build_and_optimize_motion
from momentumopt.quadruped.quadruped_wrapper import QuadrupedWrapper


class TestRobotJumps(unittest.TestCase):

    def setUp(self):
        super(TestRobotJumps, self).setUp()

    def tearDown(self):
        super(TestRobotJumps, self).tearDown()

    def optimize_the_jumping_motion(self, config_file_name, with_lqr):
        """ Build the optmization problem, optimize it and dump the data"""

        # Build the optmization problem
        motion_planner, optimized_kin_plan, optimized_motion_eff, \
            optimized_dyn_plan, dynamics_feedback, planner_setting, \
            time_vector = build_and_optimize_motion(
                config_file_name, QuadrupedWrapper,
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

        self.assertEqual(sorted(old_data), sorted(new_data))

        for key in old_data:
            assert_all_close(old_data[key], new_data[key], 1e-03)

    def load_data_from_files(self, folder_name):
        data = dict()
        for (dirpath, _, filenames) in walk(folder_name):
            for filename in filenames:
                key, _ = path.splitext(path.basename(filename))
                data[key] = np.loadtxt(path.join(dirpath, filename))
        return data

    def test_cfg_solo_jump_with_lqr(self):
        # Setup
        # get the package path
        pkg_path = path.dirname(path.abspath(path.curdir))
        # get the config folder path
        yaml_config_dir = path.join(pkg_path, "config")
        # get the config file
        yaml_file = path.join(yaml_config_dir, "cfg_solo_jump.yaml")
        # get the path to the precomputed trajectories
        precomputed_data_folder = path.join(
            pkg_path, "tests", "jump_trajectories", "solo_jump_with_lqr")
        # create an empty directory to store the data
        self.data_dir = tempfile.mkdtemp()
        # Test the setup
        self.assertTrue(path.exists(yaml_config_dir))
        self.assertTrue(path.exists(precomputed_data_folder))

        # compute the motion for the jump
        self.optimize_the_jumping_motion(
            config_file_name=yaml_file, with_lqr=True)

        # load and check the data
        self.check_generated_data(precomputed_data_folder)

    def test_cfg_solo_jump_without_lqr(self):
        # Setup
        # get the package path
        pkg_path = path.dirname(path.abspath(path.curdir))
        # get the config folder path
        yaml_config_dir = path.join(pkg_path, "config")
        # get the config file
        yaml_file = path.join(yaml_config_dir, "cfg_solo_jump.yaml")
        # get the path to the precomputed trajectories
        precomputed_data_folder = path.join(
            pkg_path, "tests", "jump_trajectories", "solo_jump_without_lqr")
        # create an empty directory to store the data
        self.data_dir = tempfile.mkdtemp()
        # Test the setup
        self.assertTrue(path.exists(yaml_config_dir))
        self.assertTrue(path.exists(precomputed_data_folder))

        # compute the motion for the jump
        self.optimize_the_jumping_motion(
            config_file_name=yaml_file, with_lqr=False)

        # load and check the data
        self.check_generated_data(precomputed_data_folder)
