import numpy as np
import casadi as ca

from quadrotor import Quadrotor

# from trajectory import Trajectory

class Optimization():
    def __init__(self, quad:Quadrotor, wp_num:int, Ns:list, tol=0.3, _tol_term=0.01):
        # Quadrotor dynamics
        self._quad = quad
        self._ddynamics = self._quad.ddynamics_dt()

        self._tol = tol
        self._tol_term = _tol_term

        # Number of gates
        self._wp_num = wp_num
        self._seg_num = len(Ns)
        assert(self._seg_num==wp_num + 1)
        self._Ns = Ns
        self._Horizon = 0
        self._N_wp_base = [0]
        # _N_wp_base: index to each segment
        # _Horizon: total sampled nodes
        for i in range(self._seg_num):
            self._N_wp_base.append(self._N_wp_base[i]+self._Ns[i])
            self._Horizon += self._Ns[i]
        print("Total points: ", self._Horizon)
        # Dimension of states
        self._X_dim = self._ddynamics.size1_in(0)
        # Dimension of controls
        self._U_dim = self._ddynamics.size1_in(1)
        # Lower and upper bounds
        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub

        # Time optimization variables
        self._DTs = ca.SX.sym('DTs', self._seg_num)
        # State optimization variables
        self._Xs = ca.SX.sym('Xs', self._X_dim, self._Horizon)
        # Control optimization variables
        self._Us = ca.SX.sym('Us', self._U_dim, self._Horizon)
        # Parameters for waypoints
        self._WPs_p = ca.SX.sym('WPs_p', 3, self._wp_num)

        # Initialization values
        self._X_init = ca.SX.sym('X_init', self._X_dim)
        self._X_end = ca.SX.sym('X_end', self._X_dim)

        # Weights for waypoint-constraint violation
        self._cost_WP_p = ca.diag([1,1,1]) # opt param
        self._cost_state = ca.diag([1,1,1,0.5,0.5,0.5,0.1,0.1,0.1,0.1,0.05,0.05,0.05]) # opt param

        self._opt_t_option = {
            'verbose': False,
            # 'ipopt.tol': 1e-5,
            # 'ipopt.acceptable_tol': 1e-3,
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0
        }

        #################################################################
        self._nlp_x_x = []
        self._nlp_lbx_x = []
        self._nlp_ubx_x = []

        self._nlp_x_u = []
        self._nlp_lbx_u = []
        self._nlp_ubx_u = []

        self._nlp_x_t = []
        self._nlp_lbx_t = []
        self._nlp_ubx_t = []

        self._nlp_g_orientation = []
        self._nlp_lbg_orientation = []
        self._nlp_ubg_orientation = []

        self._nlp_g_dyn = []
        self._nlp_lbg_dyn = []
        self._nlp_ubg_dyn = []

        self._nlp_g_wp_p = []
        self._nlp_lbg_wp_p = []
        self._nlp_ubg_wp_p = []

        self._nlp_g_quat = []
        self._nlp_lbg_quat = []
        self._nlp_ubg_quat = []

        self._nlp_p_xinit = [ self._X_init ]
        self._xinit = np.array([0,0,0, 0,0,0, 1,0,0,0, 0,0,0])
        self._nlp_p_xend = [ self._X_end ]
        self._xend = np.array([0,0,0, 0,0,0, 1,0,0,0, 0,0,0])

        self._nlp_p_wp_p = []
        self._nlp_obj_time = 0
        ###################################################################

        for i in range(self._seg_num):
            # Add the first state variables and constraints for segment i
            self._nlp_x_x += [ self._Xs[:, self._N_wp_base[i]] ]
            self._nlp_lbx_x += self._X_lb
            self._nlp_ubx_x += self._X_ub
            # Add the first control variables and constraints for segment i
            self._nlp_x_u += [ self._Us[:, self._N_wp_base[i]] ]
            self._nlp_lbx_u += self._U_lb
            self._nlp_ubx_u += self._U_ub
            # Add time varables and constraints in durations
            self._nlp_x_t += [ self._DTs[i] ]
            self._nlp_lbx_t += [0]
            self._nlp_ubx_t += [0.5]

            # Dynamic constraint as cost functions (for the first point)
            if i==0:
                dd_dyn = self._Xs[:,0]-self._ddynamics( self._X_init, self._Us[:,0], self._DTs[0])
                self._nlp_g_dyn += [ dd_dyn ] # Equality constriants
            else:
                #TODO: Problem: the second u could be wrong!!!!
                dd_dyn = self._Xs[:,self._N_wp_base[i]]-self._ddynamics( self._Xs[:,self._N_wp_base[i]-1], self._Us[:,self._N_wp_base[i]], self._DTs[i])
                self._nlp_g_dyn += [ dd_dyn ]

            # Set zero bound for dynamics constraints
            self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
            self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
            
            # Total trajectory time used as objectives
            self._nlp_obj_time += self._DTs[i]*self._Ns[i]
            
            # Add variables and constraitns for the rest of the state
            for j in range(1, self._Ns[i]):
                self._nlp_x_x += [ self._Xs[:, self._N_wp_base[i]+j] ]
                self._nlp_lbx_x += self._X_lb
                self._nlp_ubx_x += self._X_ub
                self._nlp_x_u += [ self._Us[:, self._N_wp_base[i]+j] ]
                self._nlp_lbx_u += self._U_lb
                self._nlp_ubx_u += self._U_ub
                
                dd_dyn = self._Xs[:,self._N_wp_base[i]+j]-self._ddynamics( self._Xs[:,self._N_wp_base[i]+j-1], self._Us[:,self._N_wp_base[i]+j], self._DTs[i])
                self._nlp_g_dyn += [ dd_dyn ]
                self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
                self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
    
            if i==self._seg_num-1:
              # The last point
              self._nlp_g_wp_p += [ (self._Xs[:,self._N_wp_base[i+1]-1]-self._X_end).T@(self._Xs[:,self._N_wp_base[i+1]-1]-self._X_end) ]
              self._nlp_lbg_wp_p += [0]
              self._nlp_ubg_wp_p += [ self._tol_term*self._tol_term ] # A tolerance of 0.01 is set by default
            else:
              self._nlp_g_wp_p += [ (self._Xs[:3,self._N_wp_base[i+1]-1]-self._WPs_p[:,i]).T@(self._Xs[:3,self._N_wp_base[i+1]-1]-self._WPs_p[:,i]) ]
              self._nlp_lbg_wp_p += [0]
              self._nlp_ubg_wp_p += [ self._tol*self._tol ] # A tolerance of 0.01 is set by default
              self._nlp_p_wp_p += [ self._WPs_p[:,i] ]

    def set_initial_guess(self, xut0):
        self._xut0 = xut0

    # time-optimal planner solver
    def define_opt_t(self):
        # Setup the problem with dynamics constraints
        nlp_dect = {
            'f': self._nlp_obj_time,
            'x': ca.vertcat(*(self._nlp_x_x+self._nlp_x_u+self._nlp_x_t)),
            'p': ca.vertcat(*(self._nlp_p_xinit+self._nlp_p_xend+self._nlp_p_wp_p)),
            'g': ca.vertcat(*(self._nlp_g_dyn+self._nlp_g_wp_p)),
        }
        # Warm-up or not depends on the optimization parameters
        self._opt_t_solver = ca.nlpsol('opt_t', 'ipopt', nlp_dect, self._opt_t_option)
        self._lam_x0 = np.zeros(self._opt_t_solver.size_in(6)[0])
        self._lam_g0 = np.zeros(self._opt_t_solver.size_in(7)[0])
        
    def solve_opt_t(self, xinit, xend, wp_p, warm=False):
        p = np.zeros(2*self._X_dim+3*self._wp_num)
        p[:self._X_dim] = xinit
        p[self._X_dim:2*self._X_dim] = xend
        p[2*self._X_dim:2*self._X_dim+3*self._wp_num] = wp_p
  
        # The only difference is in parameter settings
        # Use self._xut0 as initual guesses

        # x = np.array(self._xut0[: self._Horizon*self._X_dim]).reshape((self._X_dim, self._Horizon))
        # print("initial x: ", x[:3, :])
        # print("size x0: ", len(self._xut0))

        res = self._opt_t_solver(
            x0=self._xut0,
            lam_x0 = self._lam_x0,
            lam_g0 = self._lam_g0,
            lbx=(self._nlp_lbx_x+self._nlp_lbx_u+self._nlp_lbx_t),
            ubx=(self._nlp_ubx_x+self._nlp_ubx_u+self._nlp_ubx_t),
            lbg=(self._nlp_lbg_dyn+self._nlp_lbg_wp_p),
            ubg=(self._nlp_ubg_dyn+self._nlp_ubg_wp_p),
            p=p
        )

        # Get the optimized variables 
        self._xut0 = res['x'].full().flatten()
        # Lagrangian for constriants
        self._lam_x0 = res["lam_x"]
        self._lam_g0 = res["lam_g"]

        dts = self._xut0[-self._seg_num:]
        print("optimized dts: ", dts)
        x = np.array(self._xut0[: self._Horizon*self._X_dim]).reshape((self._Horizon, self._X_dim))
        u = np.array(self._xut0[self._Horizon*self._X_dim : -self._seg_num]).reshape((self._Horizon, self._U_dim))
        # print("optimized x omg: ", x[:, 10:13])
        # print("optimized x: ", x)

        # print("optimized u: ", u)
        # print("xut0: ", self._xut0)

        T = u[0,:]
        w = np.zeros(3)
        print("optimized T: ", T)
        print("optimized w: ", w)
        residual = np.matmul(self._quad._J_inv, np.array([self._quad._arm_l*(T[0]-T[1]-T[2]+T[3]), self._quad._arm_l*(-T[0]-T[1]+T[2]+T[3]), self._quad._c_tau*(T[0]-T[1]+T[2]-T[3])]))
        residual = residual - np.cross(w,np.matmul(self._quad._J,w))
        
        print("residual1: ", residual)

        T = u[1,:]
        w = x[0,10:13]
        print("optimized T: ", T)
        print("optimized w: ", w)
        residual = np.matmul(self._quad._J_inv, np.array([self._quad._arm_l*(T[0]-T[1]-T[2]+T[3]), self._quad._arm_l*(-T[0]-T[1]+T[2]+T[3]), self._quad._c_tau*(T[0]-T[1]+T[2]-T[3])]))
        residual = residual - np.cross(w,np.matmul(self._quad._J,w))
        
        print("residual1: ", residual)

        T = u[2,:]
        w = x[1,10:13]
        print("optimized T: ", T)
        print("optimized w: ", w)
        residual = np.matmul(self._quad._J_inv, np.array([self._quad._arm_l*(T[0]-T[1]-T[2]+T[3]), self._quad._arm_l*(-T[0]-T[1]+T[2]+T[3]), self._quad._c_tau*(T[0]-T[1]+T[2]-T[3])]))
        residual = residual - np.cross(w,np.matmul(self._quad._J,w))
        
        print("residual2: ", residual)
        # -cross(w,mtimes(self._quad._J,w)))

        
        return res