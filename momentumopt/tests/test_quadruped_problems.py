"""
Copyright [2017] Max Planck Society. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import unittest
from os import path
import pkg_resources
import momentumopt
from momentumopt.PyDemoMomentumopt import main as PyDemoMomentumoptMain

class TestQuadrupedMotions(unittest.TestCase):

    def setUp(self):
        yaml_config_dir = momentumopt.__file__
        yaml_config_dir = path.dirname((path.dirname(path.dirname(yaml_config_dir))))
        yaml_config_dir = path.join(yaml_config_dir, "config")
        self.yaml_config_dir = yaml_config_dir
        print (self.yaml_config_dir)
        super(TestQuadrupedMotions, self).setUp()

    def tearDown(self):
        super(TestQuadrupedMotions, self).tearDown()

    def load_motion(self, yaml_file):
        PyDemoMomentumoptMain(["-i", yaml_file], display=False)

    def test_cfg_quadruped_fast_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_fast_long_trot.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_fast_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_fast_trot.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_jump(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_jump.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_long_trot.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_slow_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_slow_trot.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_standing_long(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_standing_long.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_standing(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_standing.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_super_long_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_super_long_trot.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_trot_inplace(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_trot_inplace.yaml")
        self.load_motion(yaml_file)  
  
    def test_cfg_quadruped_trot(self):
        yaml_file = path.join(self.yaml_config_dir, "cfg_quadruped_trot.yaml")
        self.load_motion(yaml_file)
