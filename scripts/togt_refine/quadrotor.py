import numpy as np
import casadi as ca
from casadi import MX, DM, vertcat, mtimes, Function, inv, cross, sqrt, norm_2

import yaml

import time

# Quaternion Multiplication
def quat_mult(q1,q2):
    ans = ca.vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return ca.vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

def RK4(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        k1 = DT*f_c(X1,        U)
        k2 = DT*f_c(X1+0.5*k1, U)
        k3 = DT*f_c(X1+0.5*k2, U)
        k4 = DT*f_c(X1+k3,     U)
        X1 = X1+(k1+2*k2+2*k3+k4)/6
    # F = ca.Function('F', [X0, U], [X1] ,['X0', 'U'], ['X1'])
    return X1

def EulerIntegral(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        X1 = X1 + DT*f_c(X1, U)
    
    return X1

def constrain(a, lb, ub):
    if a<lb:
        a=lb
    if a>ub:
        a=ub
    return a

class PID(object):
    def __init__(self, p:float, i:float, d:float, sum_lim:float, out_lim:float):
        self._p = p
        self._i = i
        self._d = d
        
        self._sum_lim = sum_lim
        self._out_lim = out_lim
        
        self._sum = 0
        self._last_e = 0
        
    def update(self, e:float):
        self._sum += e
        self._sum = constrain(self._sum, -self._sum_lim, self._sum_lim)
        out = self._p*e + self._i*self._sum + self._d*(e-self._last_e)
        out = constrain(out, -self._out_lim, self._out_lim)
        self._last_e = e
        
        return out    

class QuadrotorSimpleModel(object):
    def __init__(self, cfg_f):
        
        self._G = 9.8066
        self._D = np.diag([0.0, 0.0, 0.0])
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 12
        self._omega_z_max = 6
        self._a_z_max = 0
        self._a_z_min = -17

        self.load(cfg_f)

        # with open(cfg_f, 'r') as f:
        #     self._cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        # if 'omega_xy_max' in self._cfg:
        #     self._omega_xy_max = self._cfg['omega_xy_max']
        # if 'omega_z_max' in self._cfg:
        #     self._omega_z_max = self._cfg['omega_z_max']
        # if 'G' in self._cfg:
        #     self._G = self._cfg['G']
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1]
        self._U_lb = [self._T_min * 4, -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._U_ub = [self._T_max * 4,  self._omega_xy_max,  self._omega_xy_max,  self._omega_z_max]
        # self._U_lb = [self._a_z_min, -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        # self._U_ub = [self._a_z_max,  self._omega_xy_max,  self._omega_xy_max,  self._omega_z_max]


    def load(self, cfg_f):
      with open(cfg_f, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if "mass" in cfg:
          self._m = cfg["mass"]
        else:
          print("No mass specified in " + cfg_f)
      
        if "omega_max" in cfg:
          omega_max = np.array(cfg["omega_max"])
          self._omega_xy_max = omega_max[0]
          self._omega_z_max = omega_max[2]
        else:
          print("No omega_max specified in " + cfg_f)

        if "thrust_min" in cfg:
          self._T_min = cfg["thrust_min"]
        else:
          print("No min thrust specified in " + cfg_f)
        if "thrust_max" in cfg:
          self._T_max = cfg["thrust_max"]
        else:
          print("No max thrust specified in " +cfg_f)

    def dynamics(self):
        # px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        # vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        # qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        
        # az_B = ca.SX.sym('az_B')
        # wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        # X = ca.vertcat(px, py, pz,
        #                vx, vy, vz,
        #                qw, qx, qy, qz)
        # # Does not consider single-thrust constraints
        # U = ca.vertcat(az_B, wx, wy, wz)

        # fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        # X_dot = ca.vertcat(
        #     vx,
        #     vy,
        #     vz,
        #     2 * (qw * qy + qx * qz) * az_B - fdrag[0],
        #     2 * (qy * qz - qw * qx) * az_B - fdrag[1],
        #     (qw * qw - qx * qx - qy * qy + qz * qz) * az_B + self._G - fdrag[2],
        #     0.5 * (-wx * qx - wy * qy - wz * qz),
        #     0.5 * (wx * qw + wz * qy - wy * qz),
        #     0.5 * (wy * qw - wz * qx + wx * qz),
        #     0.5 * (wz * qw + wy * qx - wx * qy)
        # )

        # fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])

        p = ca.MX.sym('p', 3)
        v = ca.MX.sym('v', 3)
        q = ca.MX.sym('q', 4)
        
        w = ca.MX.sym('w', 3)
        T = ca.MX.sym('T', 1)

        x = vertcat(p, v, q)
        u = vertcat(T, w)

        g = DM([0, 0, -self._G])

        # x_dot = []
        x_dot = vertcat(
          v,
          rotate_quat(q, vertcat(0, 0, T/self._m)) + g,
          0.5*quat_mult(q, vertcat(0, w)),
        )

        # x_dot = vertcat(
        #   v,
        #   rotate_quat(q, vertcat(0, 0, T)) + g,
        #   0.5*quat_mult(q, vertcat(0, w)),
        # )


        fx = ca.Function('f', [x, u], [x_dot], ['x', 'u'], ['x_dot'])

        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        # TODO: us RK4 integration
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        # State prediction
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

class Quadrotor(object):
    def __init__(self, cfg_f):
        
        self._m = 1.0         # total mass
        self._arm_l = 0.23    # arm length
        self._beta = np.pi / 4
        self._has_beta = False
        self._c_tau = 0.0133  # torque constant
        
        self._G = 9.8066
        self._J = np.diag([0.01, 0.01, 0.02])     # inertia
        self._J_inv = np.linalg.inv(self._J)
        self._D = np.diag([0.6, 0.6, 0.6])
        
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 5
        self._omega_z_max = 1
        self._T_max = 4.179
        self._T_min = 0

        self.load(cfg_f)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1,
                      -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1,
                      self._omega_xy_max, self._omega_xy_max, self._omega_z_max]

        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]

    def load(self, cfg_f):
      with open(cfg_f, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if "mass" in cfg:
          self._m = cfg["mass"]
        else:
          print("No mass specified in " + cfg_f)
        
        if "armLength" in cfg:
          self._arm_l = cfg["armLength"]
        else:
          print("No arm length specified in " + cfg_f)

        if "torCoeff" in cfg:
          self._c_tau = cfg["torCoeff"]
        else:
          print("No torque coefficient specified in " + cfg_f)
        
        if "beta" in cfg:
          self._beta = cfg["beta"]
          self._has_beta = True
        else:
          print("Use default drone frame configuration since no beta specified in " + cfg_f)
          self._has_beta = False

        self._has_tbm = False
        if "tbm_fr" in cfg and "tbm_bl" in cfg and "tbm_br" in cfg and "tbm_fl" in cfg:
          self._tbm_fr = cfg["tbm_fr"]
          self._tbm_bl = cfg["tbm_bl"]      
          self._tbm_br = cfg["tbm_br"]
          self._tbm_fl = cfg["tbm_fl"]
          self._tbm = np.matrix([self._tbm_fr, self._tbm_bl, self._tbm_br, self._tbm_fl]).T
          self._has_tbm = True
          print("Thrust mixing matrix: \n", self._tbm)

        if "inertia" in cfg:
          self._J = np.diag(cfg["inertia"])
          self._J_inv = np.linalg.inv(self._J)
        else:
          print("No inertia specified in " + cfg_f)
      
        
        if "omega_max" in cfg:
          omega_max = np.array(cfg["omega_max"])
          self._omega_xy_max = omega_max[0]
          self._omega_z_max = omega_max[2]
        else:
          print("No omega_max specified in " + cfg_f)

        if "thrust_min" in cfg:
          self._T_min = cfg["thrust_min"]
        else:
          print("No min thrust specified in " + cfg_f)
        if "thrust_max" in cfg:
          self._T_max = cfg["thrust_max"]
        else:
          print("No max thrust specified in " +cfg_f)

    def dynamics(self):
      p = ca.MX.sym('p', 3)
      v = ca.MX.sym('v', 3)
      q = ca.MX.sym('q', 4)
      w = ca.MX.sym('w', 3)
      T = ca.MX.sym('thrust', 4)

      x = vertcat(p, v, q, w)
      u = vertcat(T)

      g = DM([0, 0, -self._G])

      x_dot = []

      if self._has_tbm:
        tbm = self._tbm
        x_dot = vertcat(
          v,
          rotate_quat(q, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/self._m)) + g,
          0.5*quat_mult(q, vertcat(0, w)),
          mtimes(self._J_inv, vertcat(
            (tbm.item((0, 0))*T[0]+tbm.item((0, 1))*T[1]+tbm.item((0, 2))*T[2]+tbm.item((0, 3))*T[3]),
            (tbm.item((1, 0))*T[0]+tbm.item((1, 1))*T[1]+tbm.item((1, 2))*T[2]+tbm.item((1, 3))*T[3]),
            self._c_tau*(-T[0]-T[1]+T[2]+T[3]))
          -cross(w,mtimes(self._J,w)))
        )

        M = np.array([[1, 1, 1, 1], self._tbm])
        print("Thrust mixing matrix: \n", M)  

      elif self._has_beta:
        lsb = self._arm_l * np.sin(self._beta * np.pi / 180.0)
        lcb = self._arm_l * np.cos(self._beta * np.pi  / 180.0)
        x_dot = vertcat(
          v,
          rotate_quat(q, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/self._m)) + g,
          0.5*quat_mult(q, vertcat(0, w)),
          mtimes(self._J_inv, vertcat(
            (-lsb*T[0]+lsb*T[1]-lsb*T[2]+lsb*T[3]),
            (-lcb*T[0]+lcb*T[1]+lcb*T[2]-lcb*T[3]),
            self._c_tau*(-T[0]-T[1]+T[2]+T[3]))
          -cross(w,mtimes(self._J,w)))
        )

        M = np.array([[1, 1, 1, 1], [-lsb, lsb, -lsb, lsb], [-lcb, lcb, lcb, -lcb], [-self._c_tau, -self._c_tau, self._c_tau, self._c_tau]])
        print("Thrust mixing matrix: \n", M)
      else:
        print("Default drone frame configuration used")
        x_dot = vertcat(
          v,
          rotate_quat(q, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/self._m)) + g,
          0.5*quat_mult(q, vertcat(0, w)),
          mtimes(self._J_inv, vertcat(
            self._arm_l*(T[0]-T[1]-T[2]+T[3]),
            self._arm_l*(-T[0]-T[1]+T[2]+T[3]),
            self._c_tau*(T[0]-T[1]+T[2]-T[3]))
          -cross(w,mtimes(self._J,w)))
        )
        # M = np.array([[1, 1, 1, 1], self._arm_l*[1, -1, -1, 1], self._arm_l*[-1, -1, 1, 1], self._c_tau*[1, -1, 1, -1]])
        # print("Thrust mixing matrix: \n", M)


      fx = Function('f',  [x, u], [x_dot], ['x', 'u'], ['x_dot'])
      return fx


    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        # TODO: us RK4 integration
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        # State prediction
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

class QuadrotorSim(object):
    def __init__(self, quad:Quadrotor):
        self._quad = quad
        
        self._dyn_d = quad.ddynamics(0.001) # Continuous State Equation
        # self._dyn_d = RK4(f, 0.001, 4) # Discrete State Equation
        
        # X:=[px,py,pz, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]
        self._X = np.zeros(13)
        self._X[6] = 1
        self._T = np.zeros(4)
        
        self._pid_wx = PID(15, 0.2, 0, 7, 7)
        self._pid_wy = PID(15, 0.2, 0, 7, 7)
        self._pid_wz = PID(20, 0.3, 0, 7, 7)
        
        self._T_min = quad._T_min
        self._T_max = quad._T_max
    
    # [thrust, wx, wy, wz]
    def low_ctrl(self, U):
        wx = self._X[10]
        wy = self._X[11]
        wz = self._X[12]
        Tx = self._pid_wx.update(U[1]-wx)
        Ty = self._pid_wy.update(U[2]-wy)
        Tz = self._pid_wz.update(U[3]-wz)
        
        T1 = U[0] + Tx + Ty - Tz
        T2 = U[0] - Tx - Ty - Tz
        T3 = U[0] - Tx + Ty + Tz
        T4 = U[0] + Tx - Ty + Tz
        
        self._T[0] = constrain(T1, self._T_min, self._T_max)
        self._T[1] = constrain(T2, self._T_min, self._T_max)
        self._T[2] = constrain(T3, self._T_min, self._T_max)
        self._T[3] = constrain(T4, self._T_min, self._T_max)
    
    # [thrust, wx, wy, wz]
    def step10ms(self, U):
        for i in range(10):
            self.low_ctrl(U)
            self.step1ms()
    
#
#   T1    T3
#     \  /
#      \/
#      /\
#     /  \
#   T4    T2
#
    def step1ms(self):
        X_ = self._dyn_d(self._X, self._T)
        self._X = X_.full().flatten()
        return self._X

if __name__ == "__main__":
    quad = Quadrotor('quad.yaml')
    q_sim = QuadrotorSim(quad)
    
    U = np.array([2,1,0,0])
    for i in range(100):
        t1 = time.time()
        q_sim.step10ms(U)
        t2 = time.time()
        print(q_sim._X[6:10])
        print(t2-t1)
    
