import pymomentum as mopt
from pymomentum import PlannerSetting, ContactPlanFromFile,ContactState, DynamicsOptimizer, DynamicsState, ContactType, EffId, KinematicsSequence
from pysolver import ExitCode
import pathlib
from math import floor
import numpy as np
import matplotlib.pyplot as plt

"""
This script show how to define a problem for the dynamicOptimizer from python.
From a manually defined sequence of contact (with duration) it show how to optimize the centroidal dynamics 
and retrieve the centroidal trajectories.
It use a biped robot, walking 75cm straight with 6 steps
"""

PATH = pathlib.Path(__file__).absolute()
PATH = PATH.parent.parent / 'config'
print("PATH to config folder : ", PATH)

# load default settings:
planner_setting = PlannerSetting()
planner_setting.initialize(str(PATH / 'cfg_biped.yaml'))
# define the duration of the motion in python:
duration = 11.4
dt = planner_setting.get(mopt.PlannerDoubleParam_TimeStep)
n_time_steps = int(floor(duration/dt))
# update this values in planner_setting:
planner_setting.set(mopt.PlannerDoubleParam_TimeHorizon, duration)
planner_setting.set(mopt.PlannerIntParam_NumTimesteps, n_time_steps)

# initialize an empty contact plan:
contact_planner = ContactPlanFromFile()
contact_planner.initialize(planner_setting)
mopt_cs = contact_planner.contactSequence()

# Add contact states corresponding to the biped walk from python:
# We use a double support duration of 0.2second and single support of 1.4 seconds and a first double support phase of 1s

# Create the contact sequence for the right foot:
RF_position = np.array([0., -0.085, 0.])
mopt_cs_RF = mopt_cs.contact_states(EffId.right_foot.value())
cp = ContactState()
cp.start_time = 0.
cp.end_time = 1.
cp.contactType = ContactType.FlatContact
cp.active = True
cp.position = RF_position
mopt_cs_RF.append(cp)
#after first step
cp = ContactState()
cp.start_time = 2.4
cp.end_time = 4.2
cp.contactType = ContactType.FlatContact
cp.active = True
RF_position[0] = 0.15
cp.position = RF_position
mopt_cs_RF.append(cp)
#after second step
cp = ContactState()
cp.start_time = 5.6
cp.end_time = 7.4
cp.contactType = ContactType.FlatContact
cp.active = True
RF_position[0] = 0.45
cp.position = RF_position
mopt_cs_RF.append(cp)
#after final step
cp = ContactState()
cp.start_time = 8.8
cp.end_time = 11.41 # must be > to duration !
cp.contactType = ContactType.FlatContact
cp.active = True
RF_position[0] = 0.75
cp.position = RF_position
mopt_cs_RF.append(cp)


# Create the contact sequence for the right foot:
LF_position = np.array([0., 0.085, 0.])
mopt_cs_LF = mopt_cs.contact_states(EffId.left_foot.value())
cp = ContactState()
cp.start_time = 0.
cp.end_time = 2.6
cp.contactType = ContactType.FlatContact
cp.active = True
cp.position = LF_position
mopt_cs_LF.append(cp)
# after first step
cp = ContactState()
cp.start_time = 4
cp.end_time = 5.8
cp.contactType = ContactType.FlatContact
cp.active = True
LF_position[0] = 0.3
cp.position = LF_position
mopt_cs_LF.append(cp)
# after second step
cp = ContactState()
cp.start_time = 7.2
cp.end_time = 9.
cp.contactType = ContactType.FlatContact
cp.active = True
LF_position[0] = 0.6
cp.position = LF_position
mopt_cs_LF.append(cp)
# after final step
cp = ContactState()
cp.start_time = 10.4
cp.end_time = 11.41
cp.contactType = ContactType.FlatContact
cp.active = True
LF_position[0] = 0.75
cp.position = LF_position
mopt_cs_LF.append(cp)

# Display the final contact sequence :
#print(contact_planner.contactSequence())


# Create the initial state :
ini_state = DynamicsState()
ini_state.com = np.array([0.0, 0.0, 0.83])
ini_state.setEffPosition(EffId.right_foot, np.array([0,-0.085,0]))
ini_state.setEffPosition(EffId.left_foot, np.array([0,0.085,0]))
# define the right and left foot in contact :
ini_state.setEffActivation(EffId.right_foot, True)
ini_state.setEffActivation(EffId.left_foot, True)
# define an initial force distribution between the feet (contact normal = +z)
ini_state.setEffForce(EffId.right_foot, np.array([0,0,0.5]))
ini_state.setEffForce(EffId.left_foot, np.array([0,0,0.5]))

# set the final com position in the planner_setting:
planner_setting.set(mopt.PlannerVectorParam_CenterOfMassMotion, np.array([0.75,0,0]))

# build an empty kinematic_sequence.
# It is not used in this example but it is required to have one of the correct size for the dynamic optimization
kin_sequence = KinematicsSequence()
kin_sequence.resize(planner_setting.get(mopt.PlannerIntParam_NumTimesteps), 1)

# initialize the dynamic optimizer:
dyn_opt = DynamicsOptimizer()
dyn_opt.initialize(planner_setting)
code = dyn_opt.optimize(ini_state, contact_planner, kin_sequence, False)
print("Exit with code : ", code.name)

# retrieve the dynamicsSequence and plot it:
dyn_states = dyn_opt.dynamicsSequence().dynamics_states
assert len(dyn_states) == n_time_steps

# plot com, lmom, amom
attributes = ["com", "lmom", "amom"]
axis_names = ["x", "y", "z"]
colors = ['r', 'g', 'b']
fig, ax = plt.subplots(3, 3)
fig.canvas.set_window_title("Result of the DynamicOptimizer")
fig.suptitle("Result of the DynamicOptimizer")
timeline = np.array([dt*i for i in range(1,n_time_steps+1)])
for i in range(3):  # line = com, lmom, amom
    values = np.array([getattr(ds, attributes[i]) for ds in dyn_states])
    for j in range(3):  # col = x,y,z
        ax_sub = ax[i, j]
        ax_sub.plot(timeline, values[:, j], color=colors[j])
        ax_sub.set_xlabel('time (s)')
        ax_sub.set_ylabel(attributes[i] + " - "+axis_names[j])
        ax_sub.grid(True)

plt.show()