import numpy as np
import csv
import yaml
import math

class Trajectory():
    def __init__(self, sampled_dt=0.05, csv_traj=None, yaml_wpt=None):
        self._X_dim = 13
        self._U_dim = 4

        self._max_omg = 15
        self._max_thrust = 6.879

        self._t = np.array([])
        self._pos = np.array([])
        self._vel = np.array([])
        self._quaternion = np.array([])
        self._omega = np.array([])
        self._u = np.array([])
        self._N = 0
        self._dt = np.array([])

        self._waypoints = []
        self._durations = []
        self._sampled_dt = sampled_dt
    
        if csv_traj!="":
            self.load_csv(csv_traj)
        else:
            print("Fail to load: ", csv_traj)
            return
        if yaml_wpt!="":
            self.load_yaml(yaml_wpt)
        else:
            self._waypoints = []
            self._durations = [self._t[-1]]
            self._seg_num = 1
            self._wpt_num = 0
        self._dts = []
        self._Ns = []
        self._N_wp_base = [0]
        self._Horizon = 0
        self._dts_vec = []
        for i in range(self._seg_num):
            N = math.floor(self._durations[i] / self._sampled_dt)
            res = self._durations[i] - N * self._sampled_dt
            dt = self._sampled_dt + res / N

            self._dts.append(dt)
            self._Ns.append(N)
            self._N_wp_base.append(self._N_wp_base[i]+self._Ns[i])
            self._Horizon += self._Ns[-1]
            self._dts_vec.append([dt] * (N))
        
        self.calc_xut0()

    def load_csv(self, csv_f):
        t = []
        pos = []
        vel = []
        quaternion = []
        omega = []
        u = []
        with open(csv_f, 'r') as f:
            traj_reader = csv.DictReader(f)
            for s in traj_reader:
                t.append(float(s['t']))
                pos.append([ float(s["p_x"]), float(s["p_y"]), float(s["p_z"]) ])
                vel.append([ float(s["v_x"]), float(s["v_y"]), float(s["v_z"]) ])
                quaternion.append([ float(s["q_w"]), float(s["q_x"]), float(s["q_y"]), float(s["q_z"]) ])
                omega.append([ float(s["w_x"]), float(s["w_y"]), float(s["w_z"]) ])
                u.append([float(s["u_1"]), float(s["u_2"]), float(s["u_3"]), float(s["u_4"])])
        
        self._t = np.array(t)
        self._pos = np.array(pos)
        self._vel = np.array(vel)
        self._quaternion = np.array(quaternion)
        self._omega = np.clip(omega, -self._max_omg, self._max_omg)
        self._u = np.clip(u, 0.0, self._max_thrust)
        # self._omega = np.array(omega)
        # self._u = np.array(u)

        self._N = self._t.shape[0]-1
        assert(self._N>0)
        self._dt = self._t[1:]-self._t[:-1] 

        self._xinit = self._pos[0].tolist() + self._vel[0].tolist() + self._quaternion[0].tolist() + self._omega[0].tolist()
        self._xend = self._pos[-1].tolist() + self._vel[-1].tolist() + self._quaternion[-1].tolist() + self._omega[-1].tolist()
 

    def load_yaml(self, yaml_f):
        with open(yaml_f, 'r') as f:
            gf = yaml.load(f, Loader=yaml.FullLoader)
            # self._sampled_dt = gf["sampled_dt"]
            self._waypoints = gf["waypoints"]
            self._durations = gf["durations"]
        self._seg_num = len(self._durations)
        self._wpt_num = len(self._waypoints)

    def at(self, t):
        # x = [0] * self._X_dim
        # u = [9.8066] * self._U_dim

        idx = (np.fabs(self._t-t)).argmin()
        pos = self._pos[idx]
        quat = self._quaternion[idx]
        vel = self._vel[idx]
        omg = self._omega[idx]
        u = self._u[idx]
        x = pos.tolist() + vel.tolist() + quat.tolist() + omg.tolist()
        u = u.tolist()
        
        # print("input t: ", t)
        # print("matched t: ", self._t[idx])
        # print(x)

        # print(len(omg))
        # print(len(vel))
        # print(len(quat))

        # print(len(x))
        # print(x)

        # print(u0)



        #TODO: find the closest t in self._t and return its index
        # get x from self._pos, self._quaternion, self._vel, self._omega
        # get u from self._u 

        # idx = np.argmin(np.linalg.norm(self._pos-pos, axis=1))
        # idx -= 1 #
        # traj_seg[i,:] = self._pos[(idx+int(i*1.0))%self._N]
        # traj_v_seg[i,:] = self._vel[(idx+int(i*1.0))%self._N,:3]

        return x, u 

    def calc_xut0(self):
        self._x_base = 0
        self._u_base = self._Horizon*self._X_dim
        self._t_base = self._Horizon*(self._X_dim + self._U_dim)
        self._dim = (self._X_dim+self._U_dim)*self._Horizon+self._seg_num

        self._xut0 = np.zeros(self._dim)
        t = 0
        idx = 0
        for i in range(self._seg_num):
            dt = self._dts[i]
            self._xut0[self._t_base+i] = dt   
            for j in range(self._Ns[i]):
                t += dt
                x, u = self.at(t)
                self._xut0[self._x_base+idx*self._X_dim : self._x_base+(idx+1)*self._X_dim] = x
                self._xut0[self._u_base+idx*self._U_dim : self._u_base+(idx+1)*self._U_dim ] = u
                idx += 1



            

        # for i in range(self._Horizon):
        #     x = []
        #     u = []
        #     xut0[i*self._X_dim : (i+1)*self._X_dim] = x
        #     xut0[self._u_base+i*self._U_dim : self._u_base+(i+1)*self._U_dim ] = u



        # self._xu0 = np.zeros((self._X_dim+self._U_dim)*self._Horizon)
        # for i in range(self._Horizon):
        #     self._xu0[i*self._X_dim+6] = 1 # Quaternion
        #     # TODO: why only assign -9.8066 to the first thrust?
        #     # self._xu0[self._Horizon*self._X_dim+i*self._U_dim] = -9.8066 
        # self._xu0[self._Horizon*self._X_dim:] = 9.8066/4

    # self._xut0 = np.zeros((self._X_dim+self._U_dim)*self._Horizon+self._seg_num)
    # self._xut0[:(self._X_dim+self._U_dim)*self._Horizon] = self._xu0
    # self._xut0[(self._X_dim+self._U_dim)*self._Horizon:] = self._dt0

    def print(self):
      print("Number of points:  ", self._N)
      print("Desired sample dt: ", self._sampled_dt)
      print("Total segments: ", self._seg_num)
      print("Total waypoints: ", self._wpt_num)
      print("Total samples: ", self._Horizon)
      print("dts:  ", self._dts)
      print("Ns:  ", self._Ns)
      print("N_wp_base:  ", self._N_wp_base)
    #   print("dts_vec:  ", self._dts_vec)
      print("x1", self._xut0[: self._X_dim])
      print("x2", self._xut0[self._X_dim : 2*self._X_dim])

      print("u0", self._xut0[self._u_base : self._u_base + self._U_dim])
      print("u1", self._xut0[self._u_base + self._U_dim : self._u_base + 2*self._U_dim])

      print("uN", self._xut0[-4-self._seg_num : -self._seg_num])
      print("t", self._xut0[-self._seg_num : ])

      print("durations: ", self._durations)
      print("waypoints: ", self._waypoints)

      print("xinit: ", self._xinit)
      print("xend: ", self._xend)

    #   print(self._pos)

