import numpy as np
from matplotlib import pyplot as plt

control_freq_hz = 48
x_i = 1.0
y_i = 1.0
z_i = 1.0
R = 2
PERIOD = 30
NUM_WP = control_freq_hz*PERIOD
traj = np.zeros((3,NUM_WP))
for i in range(NUM_WP):
    traj[0,i] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+x_i
    traj[1,i] = R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+y_i
    traj[2,i] = z_i+(i/1000)

# traj = np.load('ref_helix.npy')
# print(traj[:,0])
ax = plt.axes(projection='3d')
ax.plot3D(traj[0,:],traj[1,:],traj[2,:],'g',linewidth=2)
plt.show()

np.save('ref_helix',traj)