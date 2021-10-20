from py_biconvex_mpc.ik.inverse_kinematics import InverseKinematics
import crocoddyl
import numpy as np
from pinocchio import RobotWrapper

class CrocoddylInverseKinematics(object):
    def __init__(self, model, endeff_frame_names):
        def getFrameId(name):
            idx = model.getFrameId(name)
            if idx == len(model.frames):
                raise Exception('Unknown frame name: {}'.format(name))
            return idx

        self.robot = RobotWrapper(model)
        self.model = model
        self.nv = self.model.nv
        self.data = self.robot.data
        self.mass = sum([i.mass for i in self.robot.model.inertias[1:]])
        self.base_id = self.robot.model.getFrameId('base_link')
        self.endeff_frame_names = endeff_frame_names
        self.endeff_ids = [getFrameId(name) for name in endeff_frame_names]
        self.ne = len(self.endeff_ids)
        self.stance_weight = 1e5
        self.swing_weight = 0.
        self.base_pos_regularization = 0.
        self.base_ori_regularization = 50.
        self.joint_pos_regularization = 0.2
        self.base_vel_regularization = 0.1
        self.base_ang_vel_regularization = 0.1
        self.joint_vel_regularization = 1.0
        self.state_terminal_cost = 1e-4
        self.control_terminal_cost = 1e-7

    # def update_kinematics(self, q, dq):
    #     # Update the pinocchio model.
    #     self.robot.forwardKinematics(q, dq)
    #     self.robot.computeJointJacobians(q)
    #     self.robot.framesForwardKinematics(q)

    def solve(self, dt, q, dq, com_dyn, lmom_dyn,
        amom_dyn, endeff_pos_ref, endeff_vel_ref,
        endeff_contact, joint_des, base_des_quat, contact_plan):

        num_time_steps = com_dyn.shape[0]
        end_time = num_time_steps * dt
        ik = InverseKinematics(self.model, dt, end_time)
        ik.add_com_position_tracking_task(0, end_time,
                                          com_dyn, 1e3, "com_task")
        cmom_traj = np.hstack((lmom_dyn, amom_dyn))
        ik.add_centroidal_momentum_tracking_task(0, end_time,
                                          cmom_traj, 1e3, "cent_tc")

        for eff, name in enumerate(self.endeff_frame_names):
            num_contacts = len(contact_plan[name])
            for i in range(num_contacts):
                t_start = int(contact_plan[name][i][0])
                t_end = int(contact_plan[name][i][1])
                ik.add_position_tracking_task(self.model.getFrameId(name), t_start * dt, t_end * dt,
                                              endeff_pos_ref[t_start:t_end, eff], self.stance_weight,name+"_pos")
                if i < num_contacts - 1:
                    t_start = int(contact_plan[name][i][1])
                    t_end = int(contact_plan[name][i+1][0])
                    ik.add_position_tracking_task(self.model.getFrameId(name), t_start * dt, t_end * dt,
                                                  endeff_pos_ref[t_start:t_end, eff], self.swing_weight,name+"_pos")

        x_reg = np.concatenate([q, dq])
        for it in range(num_time_steps):
            for j in range(4):
                q[j+3] = base_des_quat[j,it]
            x_reg = np.vstack((x_reg, np.concatenate([q, dq])))

        stateWeights = np.array([self.base_pos_regularization] * 3 + [self.base_ori_regularization] * 3\
                     + [self.joint_pos_regularization] * (ik.state.nv - 6) + [self.base_pos_regularization] * 3\
                     + [self.base_ang_vel_regularization] * 3 + [self.joint_vel_regularization] *(ik.state.nv - 6))
        ik.add_state_regularization_cost(0, end_time, 1e-1, x_reg, "xReg", state_weights = stateWeights)
        ik.add_ctrl_regularization_cost(0, end_time, 1e-7, "uReg")

        # setting up terminal cost model
        xRegCost = crocoddyl.CostModelState(ik.state)
        uRegCost = crocoddyl.CostModelControl(ik.state)

        comTrack = crocoddyl.CostModelCoMPosition(ik.state, com_dyn[-1], ik.state.nv)
        ik.terminalCostModel.addCost("stateReg", xRegCost, self.state_terminal_cost)
        ik.terminalCostModel.addCost("ctrlReg", uRegCost, self.control_terminal_cost)

        ik.setup_costs()
        x0 = np.concatenate([q, dq])
        print("x0", x0)
        print("x_reg", x_reg[0,:])
        ik.optimize(x0)
        xs = ik.get_states()
        us = ik.get_controls()

        return xs,us