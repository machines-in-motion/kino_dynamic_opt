### This code contains a robot wrapper with pinocchio_v2
### Author : Avadesh
### Date : 15/03/2019

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene

from quadruped_config import *

class QuadWrapper():

    def __init__(self, urdf, mesh_dirs, visualiser = True):

        self.robot_name = robot_name
        self.visualiser = visualiser

        package_dirs = str([os.path.dirname(os.path.dirname(os.path.abspath(__file__)))])
        print('urdf', urdf)
        print('package_dirs', package_dirs)
        self.robot = RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs,root_joint=pin.JointModelFreeFlyer())

        ## setting robot parameters
        self.robot.model.rotorInertia[6:] = motor_inertia
        self.robot.model.rotorGearRatio[6:] = motor_gear_ratio

        if self.visualiser:
            self.sc = GepettoGuiScene(self.robot_name)
            self.rv = RobotVisual(self.sc, self.robot_name, urdf_path, mesh_directories = mesh_dirs)

        self.set_robot_init_config()

    def set_robot_init_config(self):

        self.update_robot_config(q_init)
        if self.visualiser:
            self.rv.display(q_init)

    def update_robot_config(self, q):

        pin.forwardKinematics(self.robot.model, self.robot.data, q)






quadwrapper = QuadWrapper(urdf_path, mesh_dirs)
