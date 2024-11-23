import numpy as np
import yaml, json
import module_kinematics as kin
import module_dynamics as dyn
import module_shared as sh
from class_grid import Grid
from scipy.integrate import solve_ivp

class Vessel():

    name = None                # Name of the vessel (determines the ode to be used for the vessel object)
    vessel_id = None           # ID for the vessel object
    scale = 1                                 
    rho = 1000
    g = 9.80665
    start_velocity = np.zeros(3)
    start_angular_velocity = np.zeros(3)
    start_location = np.zeros(3)
    start_orientation = np.zeros(4)
    start_rudder_cmd = 0.0
    start_propeller_cmd = 0.0    
    
    dt = 0.1                   # Initialization of dt. Ideally we should set the value of dt for the world itself
    ode = None                 # Will be a function imported from module_dynamics
    ode_options = None

    vessel_node = None
    current_state = None
    current_state_der = None

    U_des = None
    Fn = 0.0
    length = 0.0

    grid = None
    sensors = []

    waypoints = None
    waypoints_type = None

    guidance = None
    control = None

    euler_angle_flag = True
    kinematic_kf_flag = False
    
    def __init__(self, vessel_data, vessel_id=None):

        self.vessel_id = vessel_id

        if isinstance(vessel_data, str):
            with open(vessel_data) as stream:
                vessel_data_yaml = yaml.safe_load(stream)
            self.process_vessel_input(vessel_data_yaml)
        else:
            self.process_vessel_input(vessel_data)
        
        current_state = []
        current_state.extend(self.start_velocity.tolist())
        current_state.extend(self.start_angular_velocity.tolist())
        current_state.extend(self.start_location.tolist())
        current_state.extend(self.start_orientation.tolist())
        current_state.extend([self.start_rudder_cmd, self.start_propeller_cmd])
        self.current_state = np.array(current_state)
        self.current_state_der = np.zeros(15)
    
    def process_vessel_input(self, data=None):
        
        self.name = data['name']

        if 'start_velocity' in list(data.keys()):
            self.start_velocity = np.array(data['start_velocity'])
        if 'start_angular_velocity' in list(data.keys()):
            self.start_angular_velocity = np.array(data['start_angular_velocity'])

        self.start_location = np.array(data['start_location'])
        start_orientation_euler = data['start_orientation']
        self.start_orientation = kin.eul_to_quat(start_orientation_euler)

        if 'start_rudder_cmd' in list(data.keys()):
            self.start_rudder_cmd = data['start_rudder_cmd']
        if 'start_propeller_cmd' in list(data.keys()):
            self.start_propeller_cmd = data['start_propeller_cmd']
        
        if 'propeller_diameter' in list(data.keys()):
            self.propeller_diameter = data['propeller_diameter']
        else:
            raise ValueError('Propeller diameter is a required input')

        self.free_float = data['free_floating']

        self.cog = np.array(data['COG'])
        self.gyration = np.array(data['gyration_radii'])

        if 'geometry_file' in list(data.keys()):
            self.geometry_file = data['geometry_file']
        else:
            self.geometry_file = None


        if 'density' in list(data.keys()):
            self.rho = data['density']

        if 'scale' in list(data.keys()):
            self.scale = data['scale']
        else:
            if self.name == 'makara':
                self.scale = 48.9355
            elif self.name == 'matsya':
                self.scale = 75.5
            elif self.name == 'kurma':
                self.scale = 110.0
        
        if 'length' in list(data.keys()):
            self.length = data['length']
        else:
            if self.name == 'makara':
                self.length = 154.0 / self.scale
            elif self.name == 'matsya':
                self.length = 230.0 / self.scale
            elif self.name == 'kurma':
                self.length = 320.0 / self.scale
            else:
                raise ValueError('Specified hull is not found in the library. Spcify the length of the vessel!')
        
        if 'breadth' in list(data.keys()):
            self.breadth = data['breadth']
        else:
            if self.name == 'makara':
                self.breadth = 18.78 / self.scale
            elif self.name == 'matsya':
                self.breadth = 32.2 / self.scale
            elif self.name == 'kurma':
                self.breadth = 58.0 / self.scale
            else:
                self.breadth = None
        
        if 'draft' in list(data.keys()):
            self.draft = data['draft']
        else:
            if self.name == 'makara':
                self.draft = 5.494 / self.scale
            elif self.name == 'matsya':
                self.draft = 10.8 / self.scale
            elif self.name == 'kurma':
                self.draft = 20.8 / self.scale
            else:
                self.draft = None
        
        if 'block_coefficient' in list(data.keys()):
            self.Cb = data['block_coefficient']
        else:
            if self.name == 'makara':
                self.Cb = 0.535
            elif self.name == 'matsya':
                self.Cb = 0.651
            elif self.name == 'kurma':
                self.Cb = 0.8098
            else:
                self.Cb = None

        if 'mass' in list(data.keys()):
            self.mass_input = data['mass']

        # Calculates mass in freely floating condition
        # Overrides the input mass provided if specified as freely floating

        if self.free_float and all(x is not None for x in [self.length, self.breadth, self.draft, self.Cb]):
            
            self.mass = self.rho * self.Cb * self.length * self.breadth * self.draft
        
        else:
            
            if 'mass' in list(data.keys()):
                self.mass = self.mass_input
            else:
                raise ValueError('Unable to calculate mass! Either specify mass attribute or specify length, breadth, draft and block_coefficient attributes')

        # Calculate Intertia
        self.inertia = self.mass * self.gyration * np.abs(self.gyration)

        # Probably not getting used currently ?
        self.mass_matrix = np.zeros((6,6))
        np.fill_diagonal(self.mass_matrix[0:3, 0:3], self.mass)
        self.mass_matrix[0:3, 3:6] = -self.mass * kin.Smat(self.cog)
        self.mass_matrix[3:6, 0:3] = self.mass * kin.Smat(self.cog)
        self.mass_matrix[3:6, 3:6] = self.inertia

        if 'design_froude_number' in list(data.keys()):
            self.Fn = data['design_froude_number']
            self.U_des = self.Fn * np.sqrt(self.g * self.length)
        
        # Desgin speed specification will overwrite Fn input
        if 'design_speed' in list(data.keys()):
            self.U_des = data['design_speed']
            self.Fn = self.U_des / np.sqrt(sh.g * self.length)
        
        if self.U_des is None:
            raise ValueError('Design speed is not specified but is a required quantity. Either specify design_froude_number or design_speed')
            

        if 'sensors' in list(data.keys()):
            self.sensors = []
            for sensor in data['sensors']:
                self.sensors.append(sensor)

        # If by mistake we set different dt for each vessel in world_file.yml then it could mess up the stepping of each vessel
        # When we initialize the objects of vessel class we can send 'world_data' also as an argument
        # The 'world_data' can include 'rho', 'g', 'dt' and other world parameters related to external forces
        # Currently ode_options has the world parametrs like wind, wave and current but these can change with changing environment
        # So we can separate out the constant ode_options like T_rudder,T_prop etc from the changing ones form world.

        # name of the ode function to use
        ode = data['ode']

        if ode == 'kcs_ode':
            self.ode = dyn.onrt_ode # The ODE structure does not change, only parameter values change
        elif ode == 'onrt_ode':
            self.ode = dyn.onrt_ode
        elif ode == 'kvlcc2_ode':
            self.ode = dyn.onrt_ode # The ODE structure does not change, only parameter values change
        else:
            raise ValueError('Specified ODE for vessel is not available!')
        

        deltad_max = 3 * np.pi / 180 * np.sqrt(self.length * self.scale / sh.g) / self.Fn
        T_rud = 1 / (4 * deltad_max)
        T_prop = T_rud
        
        self.ode_options = dyn.ode_options(scale=self.scale,
                            deltad_max = deltad_max,
                            T_prop = T_prop, 
                            T_rud = T_rud, 
                            nd_max=100)

        self.create_model()
        self.ode_options['L'] = self.length
        self.ode_options['U_des'] = self.U_des

        self.Up_int = 0

        if 'guidance' in data:
            self.guidance = data['guidance']
            
            if self.guidance == 'none':
                self.guidance = None

            if self.guidance == 'ilos':
                if 'waypoints' not in data:
                    raise ValueError('Waypoints need to be defined for ILOS guidance')
                self.waypoints = np.array(data['waypoints'])
                
                if 'waypoints_type' not in data:
                    self.waypoints_type = 'XYZ'
                else:
                    self.waypoints_type = data['waypoints_type']
        
        if 'control' in data:
            self.control = data['control']
        
        if 'euler_angle_flag' in data:
            self.euler_angle_flag = data['euler_angle_flag']            
        
        if 'kinematic_kf_flag' in data:
            self.kinematic_kf_flag = data['kinematic_kf_flag']
        
        
    def create_model(self):        
        self.grid = Grid(self.geometry_file, self.start_location, self.start_orientation, self.scale)
        self.grid.load_gdf()
        self.grid.create_2d_sections()
        self.calculate_hydrodynamics()
        self.cross_flow_drag()

    def calculate_hydrodynamics(self):
        
        if self.name == 'makara':
            hydra_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/ONRT/HydRA/Output_Archive/matlab/ONRT_hydra.json'
            pow_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/ONRT/pow.json'

        elif self.name == 'matsya':
            hydra_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/KCS/HydRA/Output_Archive/matlab/KCS_hydra.json'
            pow_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/KCS/pow.json'
        
        elif self.name == 'kurma':
            hydra_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/KVLCC2/HydRA/Output_Archive/matlab/KVLCC2_hydra.json'
            pow_file = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator/hullform/KVLCC2/pow.json'

        with open(hydra_file,'r') as file:
            mdict = json.load(file)

        with open(pow_file, 'r') as file:
            pow = json.load(file)

        if self.name == 'makara':
            J = np.array(pow['J'])
            Kt_port = np.array(pow['Kt_port'])
            Kt_stbd = np.array(pow['Kt_stbd'])
            Kt = Kt_port + Kt_stbd
        else:
            J = np.array(pow['J'])
            Kt = np.array(pow['Kt'])
        
        Amat = np.ones((np.size(J), 3))
        b = np.zeros((np.size(J)))
        Amat[:, 0] = J ** 2
        Amat[:, 1] = J
        b = Kt            
        coeff, residue, rank, sing_val = np.linalg.lstsq(Amat, b, rcond=-1)
        
        self.ode_options['pow_coeff'] = coeff
        
        # Note that the hydrodynamic data is for zero speed
        omg = np.array(mdict['w'])
        AM = np.array(mdict['AM'])
        BD = np.array(mdict['BD'])

        M = np.array(mdict['M'])
        C = np.array(mdict['C'])

        A_zero = AM[1, :, :, 0, 0]
        A33 = AM[1:, 2, 2, 0, 0]
        A44 = AM[1:, 3, 3, 0, 0]
        A55 = AM[1:, 4, 4, 0, 0]

        wn3_old = 0
        wn3_new = np.sqrt(C[2, 2] / (M[2, 2] + A33[0]))
        while np.abs(wn3_old - wn3_new) > 1e-6:
            wn3_old = wn3_new
            wn3_new = np.sqrt(C[2, 2] / (M[2, 2] + np.interp(wn3_old, omg[1:], A33)))
        wn3 = wn3_new

        wn4_old = 0
        wn4_new = np.sqrt(C[3, 3] / (M[3, 3] + A44[0]))
        while np.abs(wn4_old - wn4_new) > 1e-6:
            wn4_old = wn4_new
            wn4_new = np.sqrt(C[3, 3] / (M[3, 3] + np.interp(wn4_old, omg[1:], A44)))
        wn4 = wn4_new

        wn5_old = 0
        wn5_new = np.sqrt(C[4, 4] / (M[4, 4] + A55[0]))
        while np.abs(wn5_old - wn5_new) > 1e-6:
            wn5_old = wn5_new
            wn5_new = np.sqrt(C[4, 4] / (M[4, 4] + np.interp(wn5_old, omg[1:], A55)))
        wn5 = wn5_new

        A33_wn = np.interp(wn3, omg[1:], A33)
        A44_wn = np.interp(wn4, omg[1:], A44)
        A55_wn = np.interp(wn5, omg[1:], A55)

        B33_wn = np.interp(wn3, omg[1:], BD[1:, 2, 2, 0, 0])
        B44_wn = np.interp(wn4, omg[1:], BD[1:, 3, 3, 0, 0])
        B55_wn = np.interp(wn5, omg[1:], BD[1:, 4, 4, 0, 0])

        A = A_zero
        A[2:5][:, 2:5] = np.diag(np.array([A33_wn, A44_wn, A55_wn]))

        # Full scale natural periods for horizontal modes
        Tn1 = np.inf 
        Tn2 = np.inf
        Tn6 = np.inf

        # Damping ratio being assumed in the modes
        zeta1 = 0.1
        zeta2 = 0.1
        zeta6 = 0.1

        Xu = - 8 * np.pi * zeta1 * (M[0, 0] + A[0, 0]) / Tn1
        Yv = - 8 * np.pi * zeta2 * (M[1, 1] + A[1, 1]) / Tn2
        Nr = - 8 * np.pi * zeta6 * (M[5, 5] + A[5, 5]) / Tn6

        B = np.zeros((6,6))
        B[0, 0] = -Xu / (0.5 * sh.rho * ((self.length * self.scale) ** 2) * (self.U_des * np.sqrt(self.scale))) 
        B[1, 1] = -Yv / (0.5 * sh.rho * ((self.length * self.scale) ** 2) * (self.U_des * np.sqrt(self.scale)))
        B[2, 2] = B33_wn / (0.5 * sh.rho * ((self.length * self.scale) ** 2) * (self.U_des * np.sqrt(self.scale)))
        B[3, 3] = B44_wn / (0.5 * sh.rho * ((self.length * self.scale) ** 4) * (self.U_des * np.sqrt(self.scale)))
        B[4, 4] = B55_wn / (0.5 * sh.rho * ((self.length * self.scale) ** 4) * (self.U_des * np.sqrt(self.scale)))
        B[5, 5] = -Nr / (0.5 * sh.rho * ((self.length * self.scale) ** 4) * (self.U_des * np.sqrt(self.scale)))

        A[0:3][:, 0:3] = A[0:3][:, 0:3] / (0.5 * sh.rho * ((self.length * self.scale) ** 3))
        A[0:3][:, 3:6] = A[0:3][:, 3:6] / (0.5 * sh.rho * ((self.length * self.scale) ** 4))
        A[3:6][:, 0:3] = A[3:6][:, 0:3] / (0.5 * sh.rho * ((self.length * self.scale) ** 4))
        A[3:6][:, 3:6] = A[3:6][:, 3:6] / (0.5 * sh.rho * ((self.length * self.scale) ** 5))

        M_RB = self.mass_matrix.copy()
        M_RB[0:3][:, 0:3] = M_RB[0:3][:, 0:3] / (0.5 * sh.rho * (self.length ** 3))
        M_RB[0:3][:, 3:6] = M_RB[0:3][:, 3:6] / (0.5 * sh.rho * (self.length ** 4))
        M_RB[3:6][:, 0:3] = M_RB[3:6][:, 0:3] / (0.5 * sh.rho * (self.length ** 4))
        M_RB[3:6][:, 3:6] = M_RB[3:6][:, 3:6] / (0.5 * sh.rho * (self.length ** 5))

        C[0:3][:, 0:3] = C[0:3][:, 0:3] / (0.5 * sh.rho * (self.length * self.scale) * (self.U_des * np.sqrt(self.scale)) ** 2)
        C[0:3][:, 3:6] = C[0:3][:, 3:6] / (0.5 * sh.rho * ((self.length * self.scale) ** 2) * (self.U_des * np.sqrt(self.scale)) ** 2)
        C[3:6][:, 0:3] = C[3:6][:, 0:3] / (0.5 * sh.rho * ((self.length * self.scale) ** 2) * (self.U_des * np.sqrt(self.scale)) ** 2)
        C[3:6][:, 3:6] = C[3:6][:, 3:6] / (0.5 * sh.rho * ((self.length * self.scale) ** 3) * (self.U_des * np.sqrt(self.scale)) ** 2)
        
        self.ode_options['M_RB'] = M_RB
        self.ode_options['M_A'] = A
        self.ode_options['Dl'] = B
        self.ode_options['K'] = C

        self.ode_options['M_inv'] = np.linalg.inv(M_RB + A)

        self.ode_options['D_prop'] = self.propeller_diameter / self.length
        
        # Taken from accompanying excel sheet in hullform folder
        if self.name == 'makara':
            self.ode_options['rudder_area'] = 0.002415163
            self.ode_options['rudder_aspect_ratio'] = 1.25

        elif self.name == 'matsya':
            self.ode_options['rudder_area'] = 0.0026689453125
            self.ode_options['rudder_aspect_ratio'] = 1.8287037037037

        elif self.name == 'kurma':
            self.ode_options['rudder_area'] = 0.0026689453125
            self.ode_options['rudder_aspect_ratio'] = 1.8287037037037
        
        D_prop = self.ode_options['D_prop']
        A_R = self.ode_options['rudder_area']
        Lamda = self.ode_options['rudder_aspect_ratio']
        
        pow_coeff = self.ode_options['pow_coeff']

        if self.name == 'makara':
            wp = 0.355              # Effective Wake Fraction of the Propeller (taken from KCS)
            tp = 0.207              # Thrust Deduction Factor (taken from KCS)
            xp_R = -0.46104         # Rudder stock location based on geometry (calculated from Rhino)
            tR = 1 - 0.75           # Steering resistance deduction factor - taken from KCS - value modified
            aH = 0.0                # Rudder force increase factor - taken from KCS - value modified
            xp_H = xp_R             # Longitudinal coordinate of acting point of the additional lateral force -  taken from KCS - value modified (same as rudder stock)
            eta = 0.6               # Ratio of propeller diameter to height of rudder in slipstream
            eps = 1.0               # Ratio of (1 - wR) / (1 - wP)
            X_by_Dp = 0.766797661   # Full scale X/D = 4.0/5.2165
            lp_R = -0.75            # Effective longitudinal coordinate of rudder position (taken from KCS - value modified)
            gamma_R1 = 0.6          # gamma_R for b_p > 0 (formula from from KCS structure - values modified)
            gamma_R2 = 0.3          # gamma_R for b_p < 0 (formula from from KCS structure - values modified)

        elif self.name == 'matsya':

            wp = 0.355              # Effective Wake Fraction of the Propeller (taken from KCS)
            tp = 0.207              # Thrust Deduction Factor (taken from KCS)
            xp_R = -0.5             # Rudder stock location based on geometry (calculated from Rhino)
            tR = 0.250              # Steering resistance deduction factor - taken from KCS - value modified
            aH = 0.0                # Rudder force increase factor - taken from KCS - value modified
            xp_H = xp_R             # Longitudinal coordinate of acting point of the additional lateral force -  taken from KCS - value modified (same as rudder stock)
            eta = 0.6               # Ratio of propeller diameter to height of rudder in slipstream
            eps = 1.0               # Ratio of (1 - wR) / (1 - wP)
            X_by_Dp = 0.766797661   # Full scale X/D = 4.0/5.2165
            lp_R = -0.750           # Effective longitudinal coordinate of rudder position (taken from KCS - value modified)
            gamma_R1 = 0.6          # gamma_R for b_p > 0 (formula from from KCS structure - values modified)
            gamma_R2 = 0.3          # gamma_R for b_p < 0 (formula from from KCS structure - values modified)
        
        elif self.name == 'kurma':

            wp = 0.355              # Effective Wake Fraction of the Propeller (taken from KCS)
            tp = 0.220              # Thrust Deduction Factor (taken from KVLCC2 MMG Paper)
            xp_R = -0.5             # Rudder stock location based on geometry (calculated from Rhino)
            tR = 0.387              # Steering resistance deduction factor - taken from KVLCC2 MMG Paper
            aH = 0.312              # Rudder force increase factor - taken from KVLCC2 MMG Paper
            xp_H = -0.464           # Longitudinal coordinate of acting point of the additional lateral force -  taken from KVLCC2 MMG Paper
            eta = 0.6               # Ratio of propeller diameter to height of rudder in slipstream
            eps = 1.09              # Ratio of (1 - wR) / (1 - wP)
            X_by_Dp = 0.766797661   # Full scale X/D = 4.0/5.2165
            lp_R = -0.710           # Effective longitudinal coordinate of rudder position (taken from KVLCC2 MMG Paper)
            gamma_R1 = 0.640        # gamma_R for b_p > 0 (formula from from KVLCC2 MMG Paper)
            gamma_R2 = 0.395        # gamma_R for b_p < 0 (formula from from KVLCC2 MMG Paper)
        
        self.ode_options['wp'] = wp
        self.ode_options['tp'] = tp
        self.ode_options['xp_R'] = xp_R
        self.ode_options['tR'] = tR
        self.ode_options['aH'] = aH
        self.ode_options['xp_H'] = xp_H
        self.ode_options['eta'] = eta
        self.ode_options['eps'] = eps
        self.ode_options['X_by_Dp'] = X_by_Dp
        self.ode_options['lp_R'] = lp_R
        self.ode_options['gamma_R1'] = gamma_R1
        self.ode_options['gamma_R2'] = gamma_R2
         
        n_prop = (800 / 60 ) * self.length / self.U_des
        up_prop = (1 - wp)
        kappa_R = 0.5 + 0.5 * X_by_Dp / (X_by_Dp + 0.15)
        f_alp = 6.13 * Lamda / (2.25 + Lamda)
        
        Kt_up2_by_J2 = pow_coeff[0] * (up_prop ** 2) + pow_coeff[1] * n_prop * D_prop * up_prop + pow_coeff[2] * (n_prop * D_prop) ** 2
        Up_R = eps * np.sqrt( eta * (up_prop + kappa_R * (np.sqrt(up_prop ** 2 + 8/np.pi * Kt_up2_by_J2) - up_prop)) ** 2 + (1 - eta) * up_prop ** 2 )
        F_N = A_R * f_alp * (Up_R ** 2)

        self.ode_options['X_d_d'] = - (1 - tR) * F_N
        self.ode_options['Y_d'] = - (1 + aH) * F_N
        self.ode_options['N_d'] = - (xp_R + aH * xp_H) * F_N
        self.ode_options['X_n'] = 2 * (1 - tp) * (D_prop ** 2) * ((1 - wp) * D_prop * pow_coeff[1] + 2 * n_prop * (D_prop ** 2) * pow_coeff[2])
        self.ode_options['X_n_n'] = 2 * (1 - tp) * (D_prop ** 2) * (2 * (D_prop ** 2) * pow_coeff[2])
        
        if self.name == 'kurma':
            self.ode_options['R0'] = 0.022
            self.ode_options['X_v_v'] = -0.040
            self.ode_options['X_v_r'] = 0.002
            self.ode_options['X_r_r'] = 0.011
            self.ode_options['X_v_v_v_v'] = 0.771
            
            self.ode_options['Y_v'] = -0.315
            self.ode_options['Y_R'] = 0.083
            self.ode_options['Y_v_v_v'] = -1.607
            self.ode_options['Y_v_v_r'] = 0.379
            self.ode_options['Y_v_r_r'] = -0.391
            self.ode_options['Y_r_r_r'] = 0.008
            
            self.ode_options['N_v'] = -0.137
            self.ode_options['N_R'] = -0.049
            self.ode_options['N_v_v_v'] = -0.03
            self.ode_options['N_v_v_r'] = -0.294
            self.ode_options['N_v_r_r'] = 0.055
            self.ode_options['N_r_r_r'] = -0.013

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

    def step(self):  
        state = self.current_state.copy()
        tspan = (sh.current_time, (sh.current_time + sh.dt))

        if self.U_des is not None:
            state[0:3] = state[0:3] / self.U_des
            state[3:6] = state[3:6] * (self.length / self.U_des)
            state[6:9] = state[6:9] / self.length
            state[14] = state[14] * (self.length / self.U_des)

            tspan = (sh.current_time * self.U_des / self.length, (sh.current_time + sh.dt) * self.U_des / self.length)
        
        delta_c, n_c = self.vessel_node.delta_c, self.vessel_node.n_c * self.length / self.U_des
        if np.isnan(delta_c):
            delta_c = 0

        if np.isnan(n_c):
            n_c = 0

        sol = solve_ivp(self.ode, tspan, state, args=(delta_c, n_c, self.ode_options))
        new_state = np.array(sol.y)[:,-1]
        
        # Normalize the quaternion to a unit quaternion
        new_state[9:13] = new_state[9:13]/np.linalg.norm(new_state[9:13])

        # Calculate state derivative
        new_state_der = self.ode((sh.current_time + sh.dt) * self.U_des / self.length, new_state, delta_c, n_c, self.ode_options)
        
        # Dimensionalize state
        if self.U_des is not None:
            new_state[0:3] = new_state[0:3] * self.U_des
            new_state[3:6] = new_state[3:6] * (self.U_des / self.length)
            new_state[6:9] = new_state[6:9] * self.length

            new_state[14] = new_state[14] * (self.U_des / self.length)

            new_state_der[0:3] = new_state_der[0:3] * (self.U_des ** 2) / self.length
            new_state_der[3:6] = new_state_der[3:6] * (self.U_des ** 2) / (self.length ** 2)
            new_state_der[6:9] = new_state_der[6:9] * self.U_des
            
            new_state_der[14] = new_state_der[14] * (self.U_des ** 2) / (self.length ** 2)
            
            self.current_state = new_state
            self.current_state_der = new_state_der
        else:
            # self.current_state = np.array(sol.y)[:,-1]
            self.current_state = new_state
            self.current_state_der = new_state_der
                
        self.print_state()
    
    def print_state(self):

        eul = kin.quat_to_eul(self.current_state[9:13])
        str0 = f"Vessel ID                : {self.vessel_id}"
        str1 = f"Time (sec)               : {sh.current_time:.2f}"
        str2 = f"Linear Velocity (m/s)    : {self.current_state[0]:.4f}, {self.current_state[1]:.4f}, {self.current_state[2]:.4f}"
        str3 = f"Angular Velocity (rad/s) : {self.current_state[3]:.4f}, {self.current_state[4]:.4f}, {self.current_state[5]:.4f}"
        str4 = f"Linear Position  (m)     : {self.current_state[6]:.4f}, {self.current_state[7]:.4f}, {self.current_state[8]:.4f}"
        str5 = f"Orientation (deg)        : {eul[0]*180/np.pi:.2f}, {eul[1]*180/np.pi:.2f}, {eul[2]*180/np.pi:.2f}"
        str6 = f"Unit Quaternion          : {self.current_state[9]:.2f}, {self.current_state[10]:.2f}, {self.current_state[11]:.2f}, {self.current_state[12]:.2f}"
        str7 = f"Rudder Angle (deg)       : {self.current_state[13] * 180 / np.pi:.2f}"
        str8 = f"Propeller Speed (RPM)    : {self.current_state[14] * 60:.2f}\n\n"

        print(str0); print(str1); print(str2); print(str3); print(str4); print(str5); print(str6); print(str7); print(str8)

    def reset(self):
        pass