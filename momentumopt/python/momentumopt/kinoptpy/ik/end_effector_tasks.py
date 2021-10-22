## This file contains apis to create different end effector tasks
## for the inverse kinematics problem
## Author : Avadesh Meduri
## Date : 24/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

class EndEffectorTasks:

    def __init__(self):
        ## This class contains different tasks for swing foot trajectory
        ## This is inherited by the inversekinematics class
        ## Note :
            ## Paramteres such as self.dt etc.. are defined in the inverse kinematics
            ## class
        pass

    def add_position_tracking_task(self, fid, st, et, traj, wt, cost_name):
        """
        This function creates a end effector tracking task in the ddp
        Input:
            fid : the frame id of the frame that should track the trajectory
            st : start time (sec)
            et : end time (sec)
            traj : the trajectory to be tracked (must have shape ((en - sn)/dt, 3))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(np.round(st/self.dt, 2)), int(np.round(et/self.dt, 2))
        if st != et:
            for i in range(sn, en):
                Mref = crocoddyl.FrameTranslation(fid, traj[i - sn])
                # goalTrackingCost = crocoddyl.CostModelFrameTranslation(self.state, Mref, self.actuation.nu)
                goalTrackingCost = crocoddyl.CostModelFrameTranslation(self.state, Mref)
                self.rcost_model_arr[i].addCost(cost_name + "_" + str(i), goalTrackingCost, wt)
        else:
            Mref = crocoddyl.FrameTranslation(fid, traj)
            # goalTrackingCost = crocoddyl.CostModelFrameTranslation(self.state, Mref, self.actuation.nu)
            goalTrackingCost = crocoddyl.CostModelFrameTranslation(self.state, Mref)
            self.rcost_model_arr[sn].addCost(cost_name + "_" + str(st), goalTrackingCost , wt)
            
    def add_velocity_tracking_task(self, fid, st, et, traj, wt, cost_name):
        """
        This function creates a task for the end effector to track a desired velocity
        Input:
            fid : the frame id of the frame that should track the trajectory
            st : start time (sec)
            et : end time (sec)
            traj : the velocity trajectory to be tracked (must have shape ((en - sn)/dt, 6))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(np.round(st/self.dt, 2)), int(np.round(et/self.dt, 2))
        for i in range(sn, en):
            Vref = crocoddyl.FrameMotion(fid, pin.Motion(traj[i - sn]))
            # velTrackingCost = crocoddyl.CostModelFrameVelocity(self.state, Vref, self.actuation.nu)
            velTrackingCost = crocoddyl.CostModelFrameVelocity(self.state, Vref)

            self.rcost_model_arr[i].addCost(cost_name +  "_" + str(i), velTrackingCost, wt)

    def add_orientation_tracking_task(self, fid, st, et, traj, wt, cost_name):
        """
        This function creates a task for the given frame to track a given orientation
        Input:
            fid : the frame id of the frame that should track the trajectory
            st : start time (sec)
            et : end time (sec)
            traj : the orientation trajectory to be tracked (must have shape ((en - sn)/dt, 3,3))
            wt : weight of the cost
            cost_name : name of the cost
        """

        sn, en = int(np.round(st/self.dt, 2)), int(np.round(et/self.dt, 2))
        for i in range(sn, en):
            Oref = crocoddyl.FrameRotation(fid, traj[i-sn])
            # oriTrackingCost = crocoddyl.CostModelFrameRotation(self.state, Oref, self.actuation.nu)
            oriTrackingCost = crocoddyl.CostModelFrameRotation(self.state, Oref)
            self.rcost_model_arr[i].addCost(cost_name +  "_" + str(i), oriTrackingCost, wt)
