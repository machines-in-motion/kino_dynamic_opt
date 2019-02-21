import os

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *



urdf = str(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/src/urdf/quadruped.urdf')
package_dirs = [os.path.dirname(os.path.dirname(os.path.abspath(__file__)))]
#RobotWrapper.__init__(self, urdf, root_joint=se3.JointModelFreeFlyer(), package_dirs=package_dirs)
## For pinocchio_v2
robot = RobotWrapper.BuildFromURDF(urdf, package_dirs=package_dirs, root_joint=se3.JointModelFreeFlyer())
#RobotWrapper.__init__(self)

robot.initDisplay(loadModel=True)
robot.viewer.gui.addFloor('world/floor')

robot.viewer.gui.addSphere('world/blhip',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/blknee',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/bleff',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/brhip',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/brknee',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/breff',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/flhip',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/flknee',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/fleff',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/frhip',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/frknee',.03,[1.0,0.0,0.0, 1.])
robot.viewer.gui.addSphere('world/freff',.03,[1.0,0.0,0.0, 1.])
