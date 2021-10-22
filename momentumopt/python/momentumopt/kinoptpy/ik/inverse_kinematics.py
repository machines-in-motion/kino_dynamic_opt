## This file contains the implementation of the DDP based IK
## Author : Avadesh Meduri
## Date : 24/02/2021

import numpy as np

import pinocchio as pin
import crocoddyl
from . action_model import DifferentialFwdKinematics
from . end_effector_tasks import EndEffectorTasks
from . regularization_costs import RegularizationCosts
from . com_tasks import CenterOfMassTasks

class InverseKinematics(EndEffectorTasks, RegularizationCosts, CenterOfMassTasks):

    def __init__(self, rmodel, dt, T):
        """
        This class handles the inverse kinematics for the plan with crocoddyl
        Input:
            rmodel : pinocchio robot model
            dt : discrertization of time
            T : horizon of plan in seconds
        """
        self.state = crocoddyl.StateMultibody(rmodel)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)
        self.dt = dt
        self.T = T
        self.N = int(T/dt)
        # This is the array to which all costs in each time step is added
        self.rcost_model_arr = []
        for n in range(self.N):
            # rCostModel = crocoddyl.CostModelSum(self.state, self.actuation.nu)
            rCostModel = crocoddyl.CostModelSum(self.state)

            self.rcost_model_arr.append(rCostModel)
        # This is the cost array that is passed into ddp solver
        self.rcost_arr = []

        # terminal cost model
        # self.terminalCostModel = crocoddyl.CostModelSum(self.state, self.actuation.nu)
        self.terminalCostModel = crocoddyl.CostModelSum(self.state)

    def setup_costs(self):
        """
        This function makes preapres the cost arrays so that they can be provided to
        crocoddyl to be solved
        Input:
            terminalCostModel : terminal cost model for DDP
        """
        for n in range(self.N):
            runningModel = crocoddyl.IntegratedActionModelEuler(
                DifferentialFwdKinematics(self.state, self.actuation, self.rcost_model_arr[n]), self.dt)
            self.rcost_arr.append(runningModel)

        self.terminalModel = crocoddyl.IntegratedActionModelEuler(
            DifferentialFwdKinematics(self.state, self.actuation, self.terminalCostModel), 0.)

    def optimize(self, x0):

        problem = crocoddyl.ShootingProblem(x0, self.rcost_arr, self.terminalModel)
        ddp = crocoddyl.SolverDDP(problem)
        log = crocoddyl.CallbackLogger()
        ddp.setCallbacks([log,
                        crocoddyl.CallbackVerbose(),
                        ])
        # # Solving it with the DDP algorithm
        ddp.solve()

        self.opt_sol = ddp.xs
        self.us = ddp.us

    def get_controls(self):
        return self.us

    def get_states(self):
        return self.opt_sol
