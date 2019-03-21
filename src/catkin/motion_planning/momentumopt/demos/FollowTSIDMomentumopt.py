#!/usr/bin/python

'''
 Copyright [2017] Max Planck Society. All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import yaml
import numpy as np
import matplotlib.pyplot as plt

import momentumopt
from quadruped.quadruped_wrapper import QuadrupedWrapper

import time
import tsid
import os
import pinocchio as se3
import numpy.matlib as matlib
from numpy import nan
from numpy.linalg import norm as norm

np.set_printoptions(suppress=True, precision=2)

###
# Read the data files.
read_data = lambda filename: np.genfromtxt(filename)[:, 1:]

desired = {
    'q': read_data('quadruped_generalized_positions.dat'),
    'dq': read_data('quadruped_generalized_velocities.dat'),
    'ddq': read_data('quadruped_generalized_acceleration.dat'),
}


def base_pos_vec(q):
    """ Get the xyzrpy vector from the 7 dimensional xyz quaternion vector. """
    arr = q[3:7]
    quad = se3.Quaternion(arr[3], arr[0], arr[1], arr[2])

    return np.vstack([
        q[:3].reshape(-1, 1),
        se3.utils.matrixToRpy(quad.matrix())
    ])

desired['q'][:, 2] += 0.32

###
# Load the URDF model
urdf_path = '../src/urdf/quadruped.urdf'
urdf_model_path = '../src/urdf/'
robot_display = QuadrupedWrapper(urdf_path)

###
# Setup model weights and connect to display.

mu = 0.3                            # friction coefficient
fMin = 1.0                          # minimum normal force
fMax = 100.0                       # maximum normal force
contact_frames = ['BL_contact', 'BR_contact', 'FL_contact', 'FR_contact']
contactNormal = np.matrix([0., 0., 1.]).T   # direction of the normal to the contact surface

w_com = 1.0                     # weight of center of mass task
w_posture = 1e-3                # weight of joint posture task
w_base = 1.0                    # weight of joint posture task
w_forceRef = 1e-5               # weight of force regularization task

kp_contact = 10.0               # proportional gain of contact constraint
kp_com = 10.0                   # proportional gain of center of mass task
kp_posture = 10.0               # proportional gain of joint posture task
kp_base = 10.0                  # proportional gain of joint base task

dt = 0.001                      # controller time step
PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 25                  # update robot configuration in viwewer every DISPLAY_N time steps

path = urdf_path
urdf = urdf_path
vector = se3.StdVec_StdString()
vector.extend(item for item in urdf_model_path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)

# for gepetto viewer
#q = se3.getNeutralConfigurationFromSrdf(robot.model(), srdf, False)
q = desired['q'][0].reshape(-1, 1)
v = matlib.zeros(robot.nv).T

robot_display.display(q)

###
# Add tasks for the invdyn problem.
assert [robot.model().existFrame(name) for name in contact_frames]

t = 0.0                         # time
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()

# Place the robot onto the ground.
id_contact = robot_display.model.getFrameId(contact_frames[0])
# q[2] -= robot.framePosition(data, id_contact).translation[2, 0]
robot.computeAllTerms(data, q, v)

contacts = 4*[None]
for i, name in enumerate(contact_frames):
    contacts[i] =tsid.ContactPoint(name, robot, name, contactNormal, mu, fMin, fMax)
    contacts[i].setKp(kp_contact * matlib.ones(6).T)
    contacts[i].setKd(2.0 * np.sqrt(kp_contact) * matlib.ones(6).T)
    H_rf_ref = robot.framePosition(data, robot.model().getFrameId(name))
    contacts[i].setReference(H_rf_ref)
    contacts[i].useLocalFrame(False)
    invdyn.addRigidContact(contacts[i], w_forceRef, 1.0, 1)

# Adding tracking of the posture and base.
postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * matlib.ones(robot.nv-6).T)
postureTask.setKd(2.0 * np.sqrt(kp_posture) * matlib.ones(robot.nv-6).T)
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

baseTask = tsid.TaskSE3Equality("task-base", robot, "base_link")
baseKp = kp_base * matlib.ones(6).T
baseKp[3:] *= 1
baseTask.setKp(baseKp)
baseTask.setKd(2.0 * np.sqrt(kp_base) * matlib.ones(6).T)
baseTask.useLocalFrame(False)
invdyn.addMotionTask(baseTask, w_base, 1, 0.0)

###
# Solve the invdyn problem and simulate the found ddq solution forward.

# Tracking the motion from the trajectory optimizer.
N_SIMULATION = desired['q'].shape[0]             # number of time steps simulated

trajBase = tsid.TrajectoryEuclidianConstant("traj_base", q[:6])
sampleBase = trajBase.computeNext()
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q[7:])
samplePosture = trajPosture.computeNext()

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

for i in range(0, N_SIMULATION):
    time_start = time.time()
    
    sampleBase.pos(base_pos_vec(desired['q'][i]))
    sampleBase.vel(desired['dq'][i, :6].reshape(-1, 1))
    sampleBase.acc(desired['ddq'][i, :6].reshape(-1, 1))
    
    samplePosture.pos(desired['q'][i, 7:].reshape(-1, 1))
    samplePosture.vel(desired['dq'][i, 6:].reshape(-1, 1))
    samplePosture.acc(desired['ddq'][i, 6:].reshape(-1, 1))
    
    baseTask.setReference(sampleBase)
    postureTask.setReference(samplePosture)

    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0: HQPData.print_all()

    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print "[%d] QP problem could not be solved! Error code:"%(i), sol.status
        break
    
    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)
    
#     if i%PRINT_N == 0:
#         print "Time %.3f"%(t)
#         print "\tNormal forces: ",
#         for contact in contacts:
#             if invdyn.checkContact(contact.name, sol):
#                 f = invdyn.getContactForce(contact.name, sol)
#                 print "%4.1f"%(contact.getNormalForce(f)),

# #         print "\n\ttracking err %s: %.3f"%(comTask.name.ljust(20,'.'),       norm(comTask.position_error, 2))
#         print "\t||v||: %.3f\t ||dv||: %.3f"%(norm(v, 2), norm(dv))

    v_mean = v + 0.5*dt*dv
    v += dt*dv
    q = se3.integrate(robot.model(), q, dt*v_mean)
    t += dt
    
    if i%DISPLAY_N == 0: robot_display.display(q)



