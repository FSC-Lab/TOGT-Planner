import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

import sys, os
BASEPATH = os.path.abspath(__file__).split("plots/", 1)[0]+"plots/"
ROOTPATH = os.path.abspath(__file__).split("plots/", 1)[0]+".."
sys.path += [BASEPATH]
sys.path += [ROOTPATH]

directory = ROOTPATH + "/resources/trajectory"

traj_file = directory + "/togt_traj.csv"

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

# togt
matplotlib.rc('font', **{'size': 26})
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

fig = plt.figure(figsize=(13, 7))

# t = t
ts = np.linspace(t[0], t[-1], 5000)
ps = np.array([
    np.interp(ts, t, p_x),
    np.interp(ts, t, p_y),
    np.interp(ts, t, p_z)
]).T
# print("TOGT length:", np.sum(np.linalg.norm(ps[1:]-ps[:-1], axis=1)))
vs = np.array([
    np.interp(ts, t, v_x),
    np.interp(ts, t, v_y),
    np.interp(ts, t, v_z)
]).T
vs = np.linalg.norm(vs, axis=1)
v0, v1 = 6.0, np.amax(vs)
vt = np.minimum(np.maximum(vs, v0), v1)
#vt = (vt-v0) / (v1-v0)
r = 1.0 * (1-vt) + 1.0 * vt
g = 1.0 * (1-vt) + 0.0 * vt
b = 0.0 * (1-vt) + 0.0 * vt
rgb = np.array([r, g, b]).T

plt.scatter(ps[:, 0], ps[:, 1], s=5,
            c=vt, cmap=plt.cm.winter.reversed())
plt.colorbar(pad=0.0).ax.set_ylabel('Speed [m/s]')

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.grid()
plt.savefig('togt_traj.png')

plt.show()
