'''
@file test_quadruped_problems.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08

here we test all the functionalities of lqr_gain__manifold
'''

import unittest
from os import path
import pkg_resources
import eigenpy
eigenpy.switchToNumpyMatrix()

from momentumopt.quadruped.quadruped_wrapper import QuadrupedWrapper
from momentumopt.kyno_dyn_planner_solo import optimize_the_motion, build_optimization

class TestQuadrupedMotions(unittest.TestCase):

    def setUp(self):
        yaml_config_dir = path.dirname(path.abspath(path.curdir))
        yaml_config_dir = path.join(yaml_config_dir, "config")
        self.yaml_config_dir = yaml_config_dir

        print (yaml_config_dir)
        assert(path.exists(yaml_config_dir))

        # Load the robot from URDF-model
        try:
            self.robot = QuadrupedWrapper("")
            self.robot.initDisplay()
            self.robot.display(self.robot.q)
        except:
            "gepetto not initialized..."

        # Start the recording of the video
        super(TestQuadrupedMotions, self).setUp()

    def tearDown(self):
        super(TestQuadrupedMotions, self).tearDown()

    def display_motion(self, motion_planner):
        '''
        Display the motion in the gepetto_gui if possible
        '''
        try:
            motion_planner.replay_kinematics()
        except:
            "gepetto not initialized..."

    def test_motion(self, yaml_file=""):
        '''
        Optimize and test the solver
        '''
        if yaml_file == "":
            return
        
        with_lqr = True
        motion_planner = build_optimization(yaml_file, QuadrupedWrapper, with_lqr)

        # (optimized_kin_plan,
        #  optimized_motion_eff,
        #  optimized_dyn_plan,
        #  dynamics_feedback,
        #  planner_setting,
        #  time_vector) = optimize_the_motion(motion_planner, plot_com_motion=False)

        self.display_motion(motion_planner)
         

    def test_cfg_quadruped_fast_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_fast_long_trot.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_fast_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_fast_trot.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_jump(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_jump.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_long_trot.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_slow_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_slow_trot.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_standing_long(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_standing_long.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_standing(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_standing.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_super_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_super_long_trot.yaml")
        self.test_motion(yaml_file)  
  
    def test_cfg_quadruped_trot_inplace(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_trot_inplace.yaml")
        self.test_motion(yaml_file)
  
    def test_cfg_quadruped_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_trot.yaml")
        self.test_motion(yaml_file)
