## This file contains task functions for center of mass tracking
## Author : Avadesh Meduri
## Date : 25/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl

class CenterOfMassTasks:

    def __init__(self):
        ## This is inherited by the inversekinematics class
        ## Note :
            ## Paramteres such as self.dt etc.. are defined in the inverse kinematics
            ## class
        pass

    def add_com_position_tracking_task(self, st, et, traj, wt, cost_name):
        """
        This function creates a com position tracking task in the ddp
        Input:
            st : start time (sec)
            et : end time (sec)
            traj : the trajectory to be tracked (must have shape ((en - sn)/dt, 3))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(np.round(st/self.dt, 2)), int(np.round(et/self.dt, 2))
        for i in range(sn, en):
            # comTrack = crocoddyl.CostModelCoMPosition(self.state, traj[i-sn], self.actuation.nu)
            comTrack = crocoddyl.CostModelCoMPosition(self.state, traj[i-sn])
            self.rcost_model_arr[i].addCost(cost_name, comTrack, wt)


    def add_centroidal_momentum_tracking_task(self, st, et, traj, wt, cost_name):
        """
        This function creates a centroidal momentum tracking task in the ddp
        using the centroidal momentum matrix
        Input:
            st : start time (sec)
            et : end time (sec)
            traj : the trajectory to be tracked (must have shape ((en - sn)/dt, 3))
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(np.round(st/self.dt, 2)), int(np.round(et/self.dt, 2))
        for i in range(sn, en):
            # cmomTrack = crocoddyl.CostModelCentroidalMomentum(self.state, traj[i-sn], self.actuation.nu)
            cmomTrack = crocoddyl.CostModelCentroidalMomentum(self.state, traj[i-sn])
            self.rcost_model_arr[i].addCost(cost_name, cmomTrack, wt)
