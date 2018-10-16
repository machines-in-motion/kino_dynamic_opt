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
from pysolver import *
from pymomentum import *
from scipy.constants.constants import dyn

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
    ref_sequence = DynamicsSequence()
    ref_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps))

    'define contact plan'
    contact_plan = ContactPlanFromFile()
    contact_plan.initialize(planner_setting)
    contact_plan.optimize(ini_state)
    
    'optimize motion'
    dyn_optimizer = DynamicsOptimizer()
    dyn_optimizer.initialize(planner_setting, ini_state, contact_plan)
    dyn_optimizer.optimize(ref_sequence)

    print dyn_optimizer.solveTime()
    print dyn_optimizer.dynamicsSequence().dynamicsStates[planner_setting.get(PlannerIntParam_NumTimesteps)-1]
    
    print('Done...')

if __name__ == "__main__":
   main(sys.argv[1:])
