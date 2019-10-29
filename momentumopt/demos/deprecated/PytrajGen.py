### This is the demo code to run the optimizer and generate a feasible trajectory wrt
### to the given contact plan from the config file
### Author : Avadesh
## Date : 18/03/2019

import time
import os, sys

# Dynamic optimizer imports
from pysolver import *
from pymomentum import *
from pysolverlqr import *

# Kinematic optimizer imports


class MotionPlanner():

    def __init__(self, cfg_file):
        'define problem configuration'
        self.planner_setting = PlannerSetting()
        self.planner_setting.initialize(cfg_file)

        self.dynlqr_setting = SolverLqrSetting()
        self.dynlqr_setting.initialize(cfg_file, "solverlqr_dynamics")

        'define robot initial state'
        self.ini_state = DynamicsState()
        self.ini_state.fillInitialRobotState(cfg_file)

        'define reference dynamic sequence'
        #self.kin_sequence = KinematicsSequence()
        #self.kin_sequence.resize(self.planner_setting.get(PlannerIntParam_NumTimesteps),
        #                         self.planner_setting.get(PlannerIntParam_NumDofs))

        'define terrain description'
        self.terrain_description = TerrainDescription()
        self.terrain_description.loadFromFile(self.planner_setting.get(PlannerStringParam_ConfigFile))

        'define contact plan'
        self.contact_plan = ContactPlanFromFile()
        self.contact_plan.initialize(self.planner_setting)
        self.contact_plan.optimize(self.ini_state, self.terrain_description)

    def plan_motion(self):

        dyn_optimizer = DynamicsOptimizer()
        dyn_optimizer.initialize(self.planner_settings)
        ## Running only one iteration as of now
        print("Dynamic Optimizer working ....")
        dyn_optimizer.optimize(self.ini_state, self.contact_plan,
                                kin_optimizer.kinematics_sequence, False)



################ Test ##########################################################
