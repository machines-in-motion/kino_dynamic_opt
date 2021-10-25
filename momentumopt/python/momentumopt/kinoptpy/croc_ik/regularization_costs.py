## This file contains costs that regularize state and control
## Author : Avadesh Meduri
## Date : 25/02/2021

import numpy as np
import pinocchio as pin
import crocoddyl


class RegularizationCosts:

    def __init__(self):
        ## This is inherited by the inversekinematics class
        ## Note :
            ## Paramteres such as self.dt etc.. are defined in the inverse kinematics
            ## class
        pass

    def add_state_regularization_cost(self, st, et, wt, x0, cost_name, state_weights = None):
        """
        This funtions adds regularization cost on the state
        Input:
            st : start time (sec)
            et : end time (sec)
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            xRegCost = crocoddyl.CostModelState(self.state)
            if state_weights[0] == None:
                stateWeights = np.array([0.] * 3 + [500.] * 3 + [0.01] * (self.state.nv - 6) \
                             + [.01] * 3+ [10.] * 3 + [0.01] *(self.state.nv - 6))

            xRegCost = crocoddyl.CostModelState(self.state, \
                       crocoddyl.ActivationModelWeightedQuad(state_weights**2), x0[i,:])

            self.rcost_model_arr[i].addCost(cost_name+str(i), xRegCost, wt)

    def add_ctrl_regularization_cost(self, st, et, wt, cost_name):
        """
        This funtions adds regularization cost on the control
        Input:
            st : start time (sec)
            et : end time (sec)
            wt : weight of the cost
            cost_name : name of the cost
        """
        sn, en = int(st/self.dt), int(et/self.dt)
        for i in range(sn, en):
            # uRegCost = crocoddyl.CostModelControl(self.state, self.actuation.nu)
            uRegCost = crocoddyl.CostModelControl(self.state)
            self.rcost_model_arr[i].addCost(cost_name+str(i), uRegCost, wt)
