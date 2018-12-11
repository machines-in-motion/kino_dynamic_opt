#!/usr/bin/python

'''
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
'''

from pysolver import *
from pymomentum import *
from pysolverlqr import *
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import os, sys, getopt, numpy as np, pinocchio as pin

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.momentumopt.kinopt.PinocchioKinematicsInterface import PinocchioKinematicsInterface

class MotionPlanner():

    def __init__(self, cfg_file):
        'define problem configuration'
        self.planner_setting = PlannerSetting()
        self.planner_setting.initialize(cfg_file)

        'define robot initial state'
        self.ini_state = DynamicsState()
        self.ini_state.fillInitialRobotState(cfg_file)

        'define terrain description'
        self.terrain_description = TerrainDescription()
        self.terrain_description.loadFromFile(self.planner_setting.get(PlannerStringParam_ConfigFile))

        'define contact plan'
        self.contact_plan = ContactPlanFromFile()
        self.contact_plan.initialize(self.planner_setting)
        self.contact_plan.optimize(self.ini_state, self.terrain_description)


    def optimize_motion(self):
        'Dynamics optimizer'
        dyn_optimizer = DynamicsOptimizer()
        dyn_optimizer.initialize(self.planner_setting)
        
        'Kinematics optimizer'
        kin_optimizer = KinematicsOptimizer()
        kin_interface = PinocchioKinematicsInterface()
        kin_optimizer.initialize(self.planner_setting, kin_interface)
        
        'Motion optimization'
        print("DynOpt", 0)
        dyn_optimizer.optimize(self.ini_state, self.contact_plan, kin_optimizer.kinematicsSequence(), False)
        for kd_iter in range(0, self.planner_setting.get(PlannerIntParam_KinDynIterations)):
            print("KinOpt", kd_iter+1)
            kin_optimizer.optimize(self.ini_state, self.contact_plan, dyn_optimizer.dynamicsSequence(), kd_iter==0)
            print("DynOpt", kd_iter+1)
            dyn_optimizer.optimize(self.ini_state, self.contact_plan, kin_optimizer.kinematicsSequence(), True)

        
'Main function for optimization demo'
def main(argv):

    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print ('PyDemoMomentumopt.py -i <path_to_datafile>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg

    motion_planner = MotionPlanner(cfg_file)
    motion_planner.optimize_motion()

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
