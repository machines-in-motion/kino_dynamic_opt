### This code contains a robot wrapper with pinocchio_v2
### Author : Avadesh
### Date : 15/03/2019


import os

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene


class QuadWrapper():

    def __init__(self, urdf):

        package_dirs = str([os.path.dirname(os.path.dirname(os.path.abspath(__file__)))])
        print(package_dirs)
        self.robot = RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs,root_joint=pin.JointModelFreeFlyer())



urdf_path = '/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_quadruped/urdf/quadruped.urdf'
mesh_dirs = ['/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_quadruped/meshes']
# quadruped_robot = QuadWrapper(urdf_path)

sc = GepettoGuiScene("quad")
rv = RobotVisual(sc, "quadruped", urdf_path)
