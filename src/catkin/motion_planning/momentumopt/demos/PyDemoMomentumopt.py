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

import sys, getopt
import numpy as np
from pysolver import *
from pymomentum import *

'Kinematics Interface using Pinocchio'
import pinocchio
class PinocchioKinematicsInterface(KinematicsInterface):
    def __init__(self):
        KinematicsInterface.__init__(self)

    def initialize(self, planner_setting):
        print 'user initialization'

    def updateJacobians(self):
        self.centroidal_momentum_matrix = 0.1*np.ones((6,56))
        for eff_id in range(0,4):
            self.endeffector_jacobians[eff_id] = (eff_id+2.0)*0.1*np.ones((6,56))

'Main function for optimization demo'
def main(argv):
    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print 'PyDemoMomentumopt.py -i <path_to_datafile>'
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print 'PyDemoMomentumopt.py -i <path_to_datafile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg

    'define problem configuration'
    planner_setting = PlannerSetting()
    planner_setting.initialize(cfg_file)

    'define robot initial state'
    ini_state = DynamicsState()
    ini_state.fillInitialRobotState(cfg_file)

    'define reference dynamic sequence'
    kin_sequence = KinematicsSequence()
    kin_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps),
                        planner_setting.get(PlannerIntParam_NumDofs))

    'define terrain description'
    terrain_description = TerrainDescription()
    terrain_description.loadFromFile(planner_setting.get(PlannerStringParam_ConfigFile))

    'define contact plan'
    contact_plan = ContactPlanFromFile()
    contact_plan.initialize(planner_setting)
    contact_plan.optimize(ini_state, terrain_description)
     
    'optimize motion'
    dyn_optimizer = DynamicsOptimizer()
    dyn_optimizer.initialize(planner_setting)
    dyn_optimizer.optimize(ini_state, contact_plan, kin_sequence)
 
    #print dyn_optimizer.solveTime()
    #print dyn_optimizer.dynamicsSequence().dynamics_states[planner_setting.get(PlannerIntParam_NumTimesteps)-1]
    
################################################################
################################################################
    'Kinematics Interface'
    kin_interface = PinocchioKinematicsInterface()

    'Kinematics Optimizer'
    kin_optimizer = KinematicsOptimizer()
    kin_optimizer.initialize(planner_setting, kin_interface)
################################################################
################################################################
    

    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
