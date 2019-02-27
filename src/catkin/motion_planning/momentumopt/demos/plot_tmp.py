## # TEMP:

import numpy as np
from matplotlib import pyplot as plt


positions = np.loadtxt('quadruped_positions_eff.dat', dtype=float)

print(np.shape(positions))

plt.plot(positions[:, 1], label = "end_eff_x_traj")
plt.plot(positions[:, 2], label = "end_eff_y_traj")
plt.plot(positions[:, 3], label = "end_eff_z_traj")
plt.xlabel("millisec")
plt.ylabel("m")
plt.legend()
plt.grid()
plt.show()
