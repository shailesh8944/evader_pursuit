import numpy as np

class CalculateHydrodynamics:
    def __init__(self):
        pass

    def cross_flow_drag(self):
        Cd_2D = self.hoerner()
        x = self.grid.x_sec
        T = self.grid.T_sec

        v = np.linspace(-1, 1, 100)         # Non-dimensional sway velocity
        r = np.linspace(-0.7, 0.7, 100)     # Non-dimensional yaw velocity - max value corresponds to turning radius of ~ 1.5L

        v_grid, r_grid = np.meshgrid(v, r)
        v_grid = v_grid.flatten()
        r_grid = r_grid.flatten()

        v_plus_xr_grid = v_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + r_grid[:, np.newaxis] @ x[np.newaxis, :]
        Y_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x) / (self.length ** 2)
        N_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x) / (self.length ** 3)

        A = np.zeros((np.size(Y_grid), 4))
        b = np.zeros((np.size(Y_grid), 2))

        A[:, 0] = v_grid * np.abs(v_grid)
        A[:, 1] = v_grid * np.abs(r_grid)
        A[:, 2] = r_grid * np.abs(v_grid)
        A[:, 3] = r_grid * np.abs(r_grid)

        b[:, 0] = Y_grid
        b[:, 1] = N_grid
        
        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        self.ode_options['Y_v_av'] = quad_coeff[0, 0]
        self.ode_options['Y_v_ar'] = quad_coeff[1, 0]
        self.ode_options['Y_r_av'] = quad_coeff[2, 0]
        self.ode_options['Y_r_ar'] = quad_coeff[3, 0]
        self.ode_options['N_v_av'] = quad_coeff[0, 1]
        self.ode_options['N_v_ar'] = quad_coeff[1, 1]
        self.ode_options['N_r_av'] = quad_coeff[2, 1]
        self.ode_options['N_r_ar'] = quad_coeff[3, 1]

        Rn = self.U_des * self.length * 1e6
        Cf = 0.075 / ((np.log10(Rn) - 2) ** 2)
        Cr = 0.0
        k = 0.0
        Ct = Cr + Cf * (1 + k)

        self.ode_options['X_u_au'] = -self.grid.WettedArea * Ct / (self.length ** 2)

        for key, value in self.ode_options.items():
            if key.startswith(("X_", "Y_", "N_")):
                print(f"{key}: {value}")
        # exit()
    ## Function for AUV Cross Flow Drag
    def cross_flow_drag_AUV(self):
        Cd_2D = self.hoerner()
        x = self.grid.x_sec
        T = self.grid.T_sec

        v = np.linspace(-1, 1, 100)         # Non-dimensional sway velocity
        r = np.linspace(-0.7, 0.7, 100)     # Non-dimensional yaw velocity 

        w = np.linspace(-1, 1, 100)         # Non-dimensional heave velocity
        q = np.linspace(-0.7, 0.7, 100)     # Non-dimensional pitch velocity 

        v_grid, r_grid = np.meshgrid(v, r)
        v_grid = v_grid.flatten()
        r_grid = r_grid.flatten()

        w_grid, q_grid = np.meshgrid(w, q)
        w_grid = w_grid.flatten()
        q_grid = q_grid.flatten()

        v_plus_xr_grid = v_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + r_grid[:, np.newaxis] @ x[np.newaxis, :]
        Y_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x) / (self.length ** 2)
        N_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x) / (self.length ** 3)

        w_plus_xr_grid = w_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + q_grid[:, np.newaxis] @ x[np.newaxis, :]
        Z_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * w_plus_xr_grid * np.abs(w_plus_xr_grid), x) / (self.length ** 2)
        M_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * w_plus_xr_grid * np.abs(w_plus_xr_grid), x) / (self.length ** 3)

        # For sway-yaw coefficients
        A = np.zeros((np.size(Y_grid), 4))
        b = np.zeros((np.size(Y_grid), 2))

        A[:, 0] = v_grid * np.abs(v_grid)
        A[:, 1] = v_grid * np.abs(r_grid)
        A[:, 2] = r_grid * np.abs(v_grid)
        A[:, 3] = r_grid * np.abs(r_grid)

        b[:, 0] = Y_grid
        b[:, 1] = N_grid
        
        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        # Store sway-yaw coefficients
        self.ode_options['Y_v_av'] = quad_coeff[0, 0]
        self.ode_options['Y_v_ar'] = quad_coeff[1, 0]
        self.ode_options['Y_r_av'] = quad_coeff[2, 0]
        self.ode_options['Y_r_ar'] = quad_coeff[3, 0]
        self.ode_options['N_v_av'] = quad_coeff[0, 1]
        self.ode_options['N_v_ar'] = quad_coeff[1, 1]
        self.ode_options['N_r_av'] = quad_coeff[2, 1]
        self.ode_options['N_r_ar'] = quad_coeff[3, 1]

        # For heave-pitch coefficients
        A = np.zeros((np.size(Z_grid), 4))
        b = np.zeros((np.size(Z_grid), 2))

        A[:, 0] = w_grid * np.abs(w_grid)
        A[:, 1] = w_grid * np.abs(q_grid)
        A[:, 2] = q_grid * np.abs(w_grid)
        A[:, 3] = q_grid * np.abs(q_grid)

        b[:, 0] = Z_grid
        b[:, 1] = M_grid

        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        # Store heave-pitch coefficients
        self.ode_options['Z_w_aw'] = quad_coeff[0, 0]
        self.ode_options['Z_w_aq'] = quad_coeff[1, 0]
        self.ode_options['Z_q_aw'] = quad_coeff[2, 0]
        self.ode_options['Z_q_aq'] = quad_coeff[3, 0]
        self.ode_options['M_w_aw'] = quad_coeff[0, 1]
        self.ode_options['M_w_aq'] = quad_coeff[1, 1]
        self.ode_options['M_q_aw'] = quad_coeff[2, 1]
        self.ode_options['M_q_aq'] = quad_coeff[3, 1]

        Rn = self.U_des * self.length * 1e6
        Cf = 0.075 / ((np.log10(Rn) - 2) ** 2)
        Cr = 0.0
        k = 0.0
        Ct = Cr + Cf * (1 + k)

        self.ode_options['X_u_au'] = -self.grid.WettedArea * Ct / (self.length ** 2)

        for key, value in self.ode_options.items():
            if key.startswith(("X_", "Y_", "N_", "M_", "Z_")):
              print(f"{key}: {value}")
        # exit()

    def hoerner(self):

        B_by_2T = self.grid.B_sec / 2 / self.grid.T_sec

        Cd_data = np.array([[0.0108623, 1.96608], 
            [0.176606, 1.96573],
            [0.353025, 1.89756],
            [0.451863, 1.78718],
            [0.472838, 1.58374],
            [0.492877, 1.27862],
            [0.493252, 1.21082],
            [0.558473, 1.08356],
            [0.646401, 0.998631],
            [0.833589, 0.87959],
            [0.988002, 0.828415],
            [1.30807, 0.759941],
            [1.63918, 0.691442],
            [1.85998, 0.657076],
            [2.31288, 0.630693],
            [2.59998, 0.596186],
            [3.00877, 0.586846],
            [3.45075, 0.585909],
            [3.7379, 0.559877],
            [4.00309, 0.559315]])

        Cd_linear = np.interp(B_by_2T, Cd_data[:, 0], Cd_data[:, 1])
        Cy_2d = np.zeros_like(B_by_2T)
        Cy_2d = np.where(B_by_2T <= 4.00309, Cd_linear, 0.559315)
        return Cy_2d