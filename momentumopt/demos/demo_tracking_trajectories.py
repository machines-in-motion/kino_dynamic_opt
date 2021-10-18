# Follows a given kino-dyn plan using the centroidal controller,
#
# Author: Julian Viereck
# Date : 08 September 2020


from dg_demos.solo12.controllers.centroidal_controller import get_controller as ctrl_get_controller
from robot_properties_solo.config import Solo12Config

import numpy as np
import rospkg
from dg_tools.utils import *
from dg_tools.filter import ButterWorthFilter

from dynamic_graph_manager.dg_reactive_planners import EndEffectorTrajectory3D

# Use switches to toggle between the initial configuration and the reader signal.
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.switch import SwitchVector

class Solo12PlanWbc(object):
    def __init__(self, prefix):
        self.prefix = prefix
        self.readers = []
        self.switches = []

        self.config_path = os.path.join(
            rospkg.RosPack().get_path('kino_dynamic_opt'),
            'momentumopt/nodes')


        self.ctrl = ctrl_get_controller(prefix + '_wbc')

        self.kp_joint, self.kp_joint_entity = constDouble(0., prefix + '_kp_joint', True)
        self.kd_joint, self.kd_joint_entity = constDouble(0., prefix + '_kd_joint', True)

        self.default_joint_pos = [
            -0.001854, 0.936141, -1.821453,
            0.000781, 0.936383, -1.821256,
            -0.00139, -0.936056,  1.821205,
            0.001256,  -0.9358,  1.821415
        ]
        self.default_joint_vel = 12 * [0.,]

        self.des_pos_joint_sin, self.des_pos_joint = constVectorOp(
                                self.default_joint_pos, prefix + "_pos_joint_des")

        self.des_vel_joint_sin, self.des_vel_joint = constVectorOp(
                                self.default_joint_vel, prefix + "_vel_joint_des")


        self.target_pos_sin, self.target_pos = constVectorOp(12 * [0.,], prefix + "_targe_pos")
        self.target_dur_sin, self.target_dur = constVectorOp(4 * [0.,], prefix + "_targe_dur")

    def init(self, robot, vicon):
        self.robot = robot
        self.ctrl.init(robot, vicon)

    def switchReader(self, sin, default_value, config_filename, sel_value=None):
        """Creates a Reader and Switch to change between the default value and the config values"""

        # Create the reader.
        reader = Reader(config_filename)
        config_file_path = self.config_path + config_filename
        if not os.path.isfile(config_file_path):
            raise Exception('Unknown file: ' + config_file_path)
        reader.load(config_file_path)
        # The selection is inversed. Skip the timestamp.
        if not sel_value:
            reader.selec.value = ''.join(['1'] * len(default_value)) + '0'
        else:
            reader.selec.value = sel_value

        # Create the default value
        constSig = constVector(default_value)

        # Create the switch.
        self.add_switch(sin, constSig, reader.vector, config_filename)

        # Bookkeeping
        self.readers.append(reader)

    def add_switch(self, sin, sig0, sig1, name):
        switch = SwitchVector(name + "_switch")
        switch.setSignalNumber(2) # we want to switch between 2 signals
        plug(sig0, switch.sin0)
        plug(sig1, switch.sin1)
        switch.selection.value = 0

        # Connect the switch to the sin value.
        plug(switch.sout, sin)

        self.switches.append(switch)

    def switch_plan(self):
        for r in self.readers:
            r.rewind()
        for s in self.switches:
            s.selection.value = 1

    def switch_default(self):
        for s in self.switches:
            s.selection.value = 0

    def plug(self):
        ctrl = self.ctrl
        ctrl.plug()

        # Setup joint PD controller.
        robot = self.robot
        tau = (DoubleSignal(self.kp_joint) * (
                VectorSignal(self.des_pos_joint, 12) - VectorSignal(robot.device.joint_positions, 12)) +
            DoubleSignal(self.kd_joint) * (
                VectorSignal(self.des_vel_joint, 12) - VectorSignal(robot.device.joint_velocities, 12))
            )

        # Use PD controller with the centroidal controller.
        tau_res = VectorSignal(tau, 12) + VectorSignal(ctrl.control_torques, 12)
        plug(tau_res, self.robot.device.ctrl_joint_torques)

        self.switchReader(self.des_pos_joint_sin, self.default_joint_pos,
            "quadruped_generalized_positions.dat", ''.join(12 * ['1'] + 8 * ['0']))
        self.switchReader(self.des_vel_joint_sin, self.default_joint_vel,
            "quadruped_generalized_velocities.dat", ''.join(12 * ['1'] + 7 * ['0']))

        self.switchReader(ctrl.des_pos_com_sin, [0.0, 0.0, 0.2], 'quadruped_com.dat')
        self.switchReader(ctrl.des_vel_com_sin, [0.0, 0.0, 0.00], 'quadruped_com_vel.dat')
        self.switchReader(ctrl.des_fff_com_sin, [0.0, 0.0, Solo12Config.mass*9.81], 'quadruped_centroidal_forces.dat')

        self.switchReader(ctrl.des_ori_com_sin, [.0, 0.0, 0.0, 1.0], "quadruped_quaternion.dat")
        self.switchReader(ctrl.des_ang_vel_com_sin, [0.0, 0.0, 0.0], "quadruped_base_ang_velocities.dat")
        self.switchReader(ctrl.des_fft_com_sin, [0.0, 0.0, 0.], 'quadruped_centroidal_moments.dat')

        self.switchReader(ctrl.contact_plan_sin, [1, 1, 1, 1], "quadruped_contact_activation.dat")
        self.switchReader(ctrl.des_pos_eff_sin, [ 0.195,  0.147, 0.015,
                                            0.195, -0.147, 0.015,
                                            -0.195,  0.147, 0.015,
                                            -0.195, -0.147, 0.015], "quadruped_positions_abs.dat")
        self.switchReader(ctrl.des_vel_eff_sin, [0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,], "quadruped_velocities_abs.dat")


def get_controller():
    return Solo12PlanWbc(prefix="solo12")

if ('robot' in globals()) or ('robot' in locals()):
    from dg_demos.solo12.controllers import pd_controller
    from dynamic_graph_manager.vicon_sdk import ViconClientEntity

    ctrl_pd = pd_controller.get_controller()

    ctrl = Solo12PlanWbc('solo12')
    wtrl = ctrl.ctrl # whole-body-controller
    ctrl.init(robot, ViconClientEntity)

    # Offsetting the vicon base to make sure ground aligns with actual
    # ground of the robot.
    wtrl.quad_com_ctrl.vicon_offset.value = [0., 0., 0.005]

    wtrl.record_data()

    first_go_to_pd = True

    def go_pd():
        global first_go_to_pd
        if first_go_to_pd:
            ctrl_pd.plug(robot)
            first_go_to_pd = False
        else:
            # Only plug the output torque. Allows to reuse the slider offset
            # position from first calibration for the joint position.
            plug(ctrl_pd.pd.control, robot.device.ctrl_joint_torques)

            # Reset to the default plan by default as well.
            ctrl.switch_default()

    def go_warm_wbc():
        """Directly warm-start the whole body controller."""
        wtrl.kf_eff.value = 1.
        ctrl.plug()

    def go_wbc():
        ctrl.plug()
        # Need to wramp up the `wtrl.kf_eff.value` from 0 to 1.

    # Start pd controller by default.
    go_pd()
