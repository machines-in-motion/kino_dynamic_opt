## # TEMP:

import numpy as np
from matplotlib import pyplot as plt


positions = np.loadtxt('quadruped_positions_eff.dat', dtype=float)
velocities = np.loadtxt('quadruped_velocities_eff.dat', dtype=float)
com = np.loadtxt('quadruped_com.dat', dtype=float)

print(np.shape(positions))

fig, ax = plt.subplots(4,1)
ax[0].plot(positions[:, 0], label = "end_eff_x_traj")
ax[0].plot(positions[:, 1], label = "end_eff_y_traj")
ax[0].plot(positions[:, 2], label = "end_eff_z_traj")
ax[0].set_xlabel("millisec")
ax[0].set_ylabel("m")
ax[0].legend()
ax[0].grid()

ax[1].plot(velocities[:, 0], label = "end_eff_x_vel")
ax[1].plot(velocities[:, 1], label = "end_eff_y_vel")
ax[1].plot(velocities[:, 2], label = "end_eff_z_vel")
ax[1].set_xlabel("millisec")
ax[1].set_ylabel("m/s")
ax[1].legend()
ax[1].grid()

ax[2].plot(positions[:, 12], label = "end_eff_x_traj")
ax[2].plot(positions[:, 13], label = "end_eff_y_traj")
ax[2].plot(positions[:, 14], label = "end_eff_z_traj")
ax[2].set_xlabel("millisec")
ax[2].set_ylabel("m")
ax[2].legend()
ax[2].grid()

ax[3].plot(com[:, 1], label = "com_x")
ax[3].plot(com[:, 2], label = "com_y")
ax[3].plot(com[:, 3], label = "com_z")
ax[3].set_xlabel("millisec")
ax[3].set_ylabel("m")
ax[3].legend()
ax[3].grid()



plt.show()
