import pinocchio as pin
from pinocchio.utils import *

import os
from os.path import join, dirname
import rospkg

######## URDF PATH and Mesh Dirs ##############################################

urdf_path = (join(rospkg.RosPack().get_path("robot_properties_quadruped"),
                                        "urdf",
                                        "quadruped.urdf")
    )

mesh_dirs = [dirname(rospkg.RosPack().get_path("robot_properties_quadruped"))]


###############################################################################

## Quadruped config file

robot_name = "quadruped"

motor_inertia = 0.0000045

motor_gear_ratio = 9.0

motor_torque_constant = 0.025

max_current = 12.0

max_torque = motor_torque_constant * max_current

nb_joints = 8

end_effectors = ["FL", "FR", "HL", "HR"]


q_init = zero(15)
dq_init = zero(15)

q_init[0] = 0.2
q_init[1] = 0.0
q_init[2] = 0.24
q_init[6] = 1.
for i in range(4):
    q_init[7 + 2 * i] = 0.8
    q_init[8 + 2 * i] = -1.6
