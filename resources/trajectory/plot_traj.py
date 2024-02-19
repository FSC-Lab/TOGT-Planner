import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

import sys, os
directory = os.getcwd()

traj_file = directory + "/traj.csv"

data_ocp = np.genfromtxt(traj_file, dtype=float, delimiter=',', names=True)

t = data_ocp['t']
p_x = data_ocp['p_x']
p_y = data_ocp['p_y']
p_z = data_ocp['p_z']
q_w = data_ocp['q_w']
q_x = data_ocp['q_x']
q_y = data_ocp['q_y']
q_z = data_ocp['q_z']
v_x = data_ocp['v_x']
v_y = data_ocp['v_y']
v_z = data_ocp['v_z']
w_x = data_ocp['w_x']
w_y = data_ocp['w_y']
w_z = data_ocp['w_z']
u_1 = data_ocp['u_1']
u_2 = data_ocp['u_2']
u_3 = data_ocp['u_3']
u_4 = data_ocp['u_4']

matplotlib.rc('font', **{'size': 24})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

fig_pos_xy = plt.figure(1, (6,6))
axhxy = fig_pos_xy.gca()
axhxy.plot(p_x, p_y)
axhxy.set_title('')
axhxy.grid(True)
fig_pos_xy.tight_layout()
plt.show()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

t_max = t[-1]

# ax1.plot(t, u_1)
# ax1.plot(t, u_2)
# ax1.plot(t, u_3)
# ax1.plot(t, u_4)
ax1.plot(t, u_1+u_2+u_3+u_4)

ax1.set_xlim([0.0, t_max])
ax1.grid()
ax1.set_xticklabels('')

ax2.plot(t, w_x)
ax2.plot(t, w_y)
ax2.plot(t, w_z)

ax2.set_xlim([0.0, t_max])
ax2.grid()

ax2.set_xlabel('Time [s]')
ax1.set_ylabel('Thrust [N]')
ax2.set_ylabel('Omega [rad/s]')

bbox_props = dict(boxstyle="round,pad=0.2", edgecolor="gray", facecolor="white", alpha=0.7)
ax1.annotate('Thrust', xy=(16.0, 0.8), bbox=bbox_props)
ax2.annotate('Bodyrate', xy=(16.0, 0.8), bbox=bbox_props)

plt.tight_layout()

# plt.savefig('plot_compare_thrust.pdf')
plt.show()



# fig_pos_xz = plt.figure(1, (3,3))
# axhxz = fig_pos_xz.gca()
# axhxz.plot(p_x, p_z)
# axhxz.set_title('')
# axhxz.grid(True)
# fig_pos_xz.tight_layout()
# plt.show()

# #np.savetxt('result_loop3_higher_1.csv', np.transpose([
# #    t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,u_1,u_2,u_3,u_4,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z
# #]), delimiter=',', header='t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,u_1,u_2,u_3,u_4,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z')

# print(t[-1])
# import matplotlib.pyplot as plt
# #plt.plot(t, u_1+u_2+u_3+u_4)
# #plt.plot(t, (v_x**2+v_y**2+v_z**2)**0.5)
# #plt.plot(t, (w_x**2+w_y**2+w_z**2)**0.5)
# plt.plot(t, w_y)
# plt.show()
