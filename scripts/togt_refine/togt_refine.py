import numpy as np
import casadi as ca

import sys, os
BASEPATH = os.path.abspath(__file__).split("togt_refine/", 1)[0]+"togt_refine/"
ROOTPATH = os.path.abspath(__file__).split("togt_refine/", 1)[0]+".."
sys.path += [BASEPATH]
sys.path += [ROOTPATH]

from optimization import Optimization
from quadrotor import Quadrotor
from trajectory import Trajectory
import csv
def save_traj(res, opt: Optimization, csv_f):
    with open(csv_f, 'w') as f:
        traj_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #TODO: modify it to match the agilicious format csv
        labels = ['t',
                  "p_x", "p_y", "p_z",
                  "v_x", "v_y", "v_z",
                  "q_w", "q_x", "q_y", "q_z",
                  "w_x", "w_y", "w_z",
                  "a_lin_x", "a_lin_y", "a_lin_z",
                  "a_rot_x", "a_rot_y", "a_rot_z",
                  "u_1", "u_2", "u_3", "u_4",
                  "jerk_x", "jerk_y", "jerk_z",
                  "snap_x", "snap_y", "snap_z"]
        traj_writer.writerow(labels)
        x = res['x'].full().flatten()
        
        t = 0
        s = opt._xinit
        u = x[opt._Horizon*opt._X_dim: opt._Horizon*opt._X_dim+opt._U_dim]
        u_last = u

        ###
        a_lin = [0,0,0]
        a_rot = [0,0,0]
        jerk = [0,0,0]
        snap = [0,0,0]

        traj_writer.writerow([t, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], a_lin[0], a_lin[1], a_lin[2], a_rot[0], a_rot[1], a_rot[2], u[0], u[1], u[2], u[3], jerk[0], jerk[1], jerk[2], snap[0], snap[1], snap[2]])
        for i in range(opt._seg_num):
            # Optimized time gap
            dt = x[-(opt._seg_num)+i]
            for j in range(opt._Ns[i]):
                idx = opt._N_wp_base[i]+j
                t += dt
                s = x[idx*opt._X_dim: (idx+1)*opt._X_dim]
                if idx != opt._Horizon-1:
                    u = x[opt._Horizon*opt._X_dim+(idx+1)*opt._U_dim: opt._Horizon*opt._X_dim+(idx+2)*opt._U_dim]
                    u_last = u
                else:
                    u = u_last
                traj_writer.writerow([t, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], a_lin[0], a_lin[1], a_lin[2], a_rot[0], a_rot[1], a_rot[2], u[0], u[1], u[2], u[3], jerk[0], jerk[1], jerk[2], snap[0], snap[1], snap[2]])
        print("--------------------------")
        print("lap time: ", t)

def refine(quad, planner, tol, tol_term, desired_dt):
    quad = Quadrotor(ROOTPATH+'/parameters/'+quad+'/'+quad+'_quad.yaml')
    togt = Trajectory(desired_dt, ROOTPATH+"/resources/trajectory/"+planner+"_traj.csv", ROOTPATH+"/resources/trajectory/"+planner+"_wpt.yaml")
    togt.print()

    wp_opt = Optimization(quad, togt._wpt_num, togt._Ns, tol, tol_term)
    wp_opt.set_initial_guess(togt._xut0)
    wp_opt._xinit = togt._xinit
    wp_opt._xend = togt._xend

    print("\n\nTime optimization start ......\n")
    wp_opt.define_opt_t()
    res_t = wp_opt.solve_opt_t(togt._xinit, togt._xend, np.array(togt._waypoints).flatten())

    save_traj(res_t, wp_opt, ROOTPATH+"/resources/trajectory/"+planner+"_refined_traj.csv")

if __name__ == "__main__":
    quad = 'cpc'
    planner = 'togt'
    tol= 0.01
    tol_term = 0.05
    desired_dt = 0.02
    refine(quad, planner, tol, tol_term, desired_dt)
