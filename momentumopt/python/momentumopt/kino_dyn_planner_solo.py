#! /usr/bin/python

'''
@file PyDemoMomentumopt.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

import time
from pysolver import *
from pymomentum import *
from pysolverlqr import *
from pinocchio.utils import *
import os, sys, getopt, numpy as np, pinocchio as pin

from momentumopt.kinoptpy.momentum_kinematics_optimizer import MomentumKinematicsOptimizer
from momentumopt.motion_execution import MotionExecutor
from momentumopt.kinoptpy.create_data_file import create_file, create_qp_files, create_lqr_files

from momentumopt.motion_planner import MotionPlanner
from .quadruped.quadruped_wrapper import QuadrupedWrapper, Quadruped12Wrapper

import matplotlib.pyplot as plt

def parse_arguments(argv):
    cfg_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:m",["ifile=", "solo12", "disable_lqr"])
    except getopt.GetoptError:
        print ('python kino_dyn_planner.py -i <path_to_datafile>')
        sys.exit(2)

    RobotWrapper = QuadrupedWrapper
    with_lqr = True

    for opt, arg in opts:
        if opt == '-h':
            print ('PyDemoMomentumopt.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            cfg_file = arg
        elif opt in ("--solo12"):
            RobotWrapper = Quadruped12Wrapper
        elif opt in ("--disable_lqr"):
            with_lqr = False

    print(opts)
    print(cfg_file)

    if not os.path.exists(cfg_file):
        raise RuntimeError("The config file " + cfg_file + " does not exist.")

    return cfg_file, RobotWrapper, with_lqr

def build_optimization(cfg_file, RobotWrapper, with_lqr):
    """
    Build the optimization problem
    """
    # create the planner
    motion_planner = MotionPlanner(cfg_file, MomentumKinematicsOptimizer, RobotWrapper, with_lqr)

    # load all the parameters of the planner
    etg = motion_planner.kin_optimizer.endeff_traj_generator
    etg.z_offset = motion_planner.planner_setting.get(PlannerDoubleParam_SwingTrajViaZ)
    motion_planner.kin_optimizer.inv_kin.w_lin_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightLinMomentumTracking)
    motion_planner.kin_optimizer.inv_kin.w_ang_mom_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightAngMomentumTracking)
    motion_planner.kin_optimizer.inv_kin.w_endeff_contact = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffContact)
    motion_planner.kin_optimizer.inv_kin.w_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_WeightEndEffTracking)
    motion_planner.kin_optimizer.inv_kin.p_endeff_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainEndEffTracking)
    motion_planner.kin_optimizer.inv_kin.p_com_tracking = motion_planner.planner_setting.get(PlannerDoubleParam_PGainComTracking)
    motion_planner.kin_optimizer.inv_kin.w_joint_regularization = motion_planner.planner_setting.get(PlannerDoubleParam_WeightJointReg)
    motion_planner.kin_optimizer.reg_orientation = motion_planner.planner_setting.get(PlannerDoubleParam_PGainOrientationTracking)
    motion_planner.kin_optimizer.reg_joint_position = motion_planner.planner_setting.get(PlannerDoubleParam_PGainPositionTracking)

    return motion_planner

def optimize_motion(motion_planner, plot_com_motion=False):
    """
    Optimize the motion using the kino-dyn optimizer.
    For the dynamics we use the centroidal dynamics solver from this package.
    Fro the Kinematics we use a python written kinematics solver.
    """

    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector = \
      motion_planner.optimize_motion(plot_com_motion)

    # Optimize the dynamic and kinematic motion.
    return optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
           dynamics_feedback, planner_setting, time_vector


def build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr, plot_com_motion=False):
    """ Build the optimization problem and solve it in one go."""
    motion_planner = build_optimization(cfg_file, RobotWrapper, with_lqr)
    optimized_kin_plan, optimized_motion_eff, optimized_dyn_plan, \
      dynamics_feedback, planner_setting, time_vector = \
          optimize_motion(motion_planner, plot_com_motion)

    return motion_planner, optimized_kin_plan, optimized_motion_eff, \
           optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector


def main(argv):
    """
    Main function for optimization demo
    """
    # Get the arguments
    cfg_file, RobotWrapper, with_lqr = parse_arguments(argv)

    # Compute the motion
    (motion_planner, optimized_kin_plan,
     optimized_motion_eff,
     optimized_dyn_plan,
     dynamics_feedback,
     planner_setting,
     time_vector) = build_and_optimize_motion(cfg_file, RobotWrapper, with_lqr)

    # Display the motion
    display = False
    if(display): # Display the Center of mass motion
        motion_planner.plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
        # for i in range(len(time_vector)):
        #     print "\n t:",time_vector[i],"\n"
        #     print dynamics_feedback.forceGain(i)
        # motion_planner.plot_centroidal()

    # Create configuration and velocity file from motion plan for dynamic graph
    try:
        print("Replay the kinematics.")
        # motion_planner.replay_kinematics()
        motion_planner.replay_kinematics_meshcat()
    except:
        "gepetto not initialized..."


    # Dump the computed trajectory in a files (should follow the dynamic graph format)
    motion_planner.save_files()

    if(display): # plot trajectories
        motion_planner.plot_foot_traj()
        motion_planner.plot_joint_trajecory()
        motion_planner.plot_com_motion(optimized_dyn_plan.dynamics_states, optimized_kin_plan.kinematics_states)
        #motion_planner.plot_base_trajecory()

    # Potentially simulate the motion
    simulation = False
    if simulation:
        motion_executor = MotionExecutor(optimized_kin_plan, optimized_dyn_plan, dynamics_feedback, planner_setting, time_vector)
        motion_executor.execute_motion(plotting=False, tune_online=False)

    print('Done...')

if __name__ == "__main__":
    main(sys.argv[1:])
