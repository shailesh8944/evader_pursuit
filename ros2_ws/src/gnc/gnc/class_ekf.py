import numpy as np
import scipy
import warnings
from ros2_ws.src.mav_simulator.mav_simulator.module_kinematics import clip, ssa

class EKF():   
     
    def __init__(
            self, 
            sampling_rate, 
            n_states=15, 
            n_inp=1, 
            pro_noise_cov=np.eye(1), 
            imu_sampling_rate=100.0,
            position_sampling_rate=1.0,
            imu_cutoff_freq=1.0, 
            position_cutoff_freq=1.0,
            filter_acc=False, 
            filter_ang_vel=False, 
            filter_euler=False, 
            filter_position=False,
            filter="butterworth"
        ):

        self.sampling_rate = sampling_rate
        self.dt = 1/self.sampling_rate
        self.t = 0.0
        
        # Filter flags
        self.filter_acc = filter_acc
        self.filter_ang_vel = filter_ang_vel
        self.filter_euler = filter_euler
        self.filter_position = filter_position
        
        # Initialize Butterworth filter if any filtering is enabled
        if any([filter_acc, filter_ang_vel, filter_euler]):
            if filter == "butterworth": 
                self.imu_filter = ButterworthFilter(
                                    sampling_rate=imu_sampling_rate, 
                                    cutoff_freq=imu_cutoff_freq, 
                                    order=2
                                )
            elif filter == "savgol":
                self.imu_filter = SavGolFilter(
                                    window_length=11,
                                    poly_order=3
                                )
        
        if filter_position:
            if filter == "butterworth": 
                self.position_filter = ButterworthFilter(
                                            sampling_rate=position_sampling_rate, 
                                            cutoff_freq=position_cutoff_freq, 
                                            order=2
                                        )  
            elif filter == "savgol":
                self.position_filter = SavGolFilter(
                                            window_length=11,
                                            poly_order=3
                                        )
        
        self.n_states = n_states
        self.n_inp = n_inp
        self.n_pro_noise = pro_noise_cov.shape[0]

        self.A = np.eye(self.n_states)
        self.B = np.zeros((self.n_states, self.n_inp))
        self.E = np.zeros((self.n_states, self.n_pro_noise))
        self.Q = pro_noise_cov
        
        self.x0 = np.zeros((self.n_states, 1))
        self.P0 = 10000*np.eye(self.n_states)
        
        self.x = self.x0
        self.P = self.P0

        self.debug = False

    def discretize(self, discrete_flag=False):
        if discrete_flag:
            self.Ad = self.A
            self.Bd = self.B
            self.Ed = self.E
        else:
            if np.abs(np.linalg.det(self.A)) < 1e-6:
                self.Ad = np.eye(self.n_states) + self.A * self.dt
                self.Bd = self.B * self.dt
                self.Ed = self.E * self.dt
            else:
                self.Ad = scipy.linalg.expm(self.A * self.dt)
                self.Bd = np.linalg.inv(self.A) @ (self.Ad - np.eye(self.n_states)) @ self.B
                self.Ed = np.linalg.inv(self.A) @ (self.Ad - np.eye(self.n_states)) @ self.E
            
    def jacobian(self, fun, x0):
        f0 = fun(x0)
        jacob = np.zeros((np.size(f0),np.size(x0)))
        eps = 1e-4

        for i in range(np.size(x0)):
            x2 = np.copy(x0)
            x2[i] = x2[i] + eps

            x1 = np.copy(x0)
            x1[i] = x1[i] - eps

            f1 = fun(x1)
            f2 = fun(x2)
            jacob[:, i] = (f2 - f1) / (2 * eps)
        
        return jacob
    
    def predict(self, u, A=None, B=None, E=None, plant_model=None, discrete_flag=False, threshold=None):

        if A is not None:
            if A.shape == self.A.shape:
                self.A = A
            else:
                raise ValueError(f"Inconsistent size of matrix A: Expected {self.A.shape} but found {A.shape}")
        else:
            if plant_model is not None:
                self.A = self.jacobian(lambda x: plant_model(self.t, x, u=None), self.x)
        
        if B is not None:
            if B.shape == self.B.shape:
                self.B = B
            else:
                raise ValueError(f"Inconsistent size of matrix B: Expected {self.B.shape} but found {B.shape}")
        else:
            u = 0.0
            self.B = np.zeros((self.n_states, 1))
        
        if E is not None:
            if E.shape == self.E.shape:
                self.E = E
            else:
                raise ValueError(f"Inconsistent size of matrix E: Expected {self.E.shape} but found {E.shape}")
        
        self.discretize(discrete_flag=discrete_flag)

        
        if plant_model is None:
            if self.n_inp == 1:
                change = self.Ad @ self.x + self.Bd * u - self.x
            else:
                change = self.Ad @ self.x + self.Bd @ u - self.x
        else:
            # 4th order Runge-Kutta integration
            k1 = plant_model(self.t, self.x[:, 0], u)
            k2 = plant_model(self.t + self.dt/2, self.x[:, 0] + self.dt/2 * k1, u) 
            k3 = plant_model(self.t + self.dt/2, self.x[:, 0] + self.dt/2 * k2, u)
            k4 = plant_model(self.t + self.dt, self.x[:, 0] + self.dt * k3, u)
            
            x_next = self.x[:, 0] + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            change = x_next[:, np.newaxis] - self.x            
        
        if threshold is not None:
            for i in range(len(change)):
                change[i, 0] = clip(change[i, 0], threshold[i])
        
        self.x = self.x + change
        self.P = self.Ad @ self.P @ self.Ad.T + self.Ed @ self.Q @ self.Ed.T
        self.t = self.t + self.dt
        for i in range(3,6):
            self.x[i] = ssa(self.x[i])

    def correct(self, y, Cd, R, meas_model=None, threshold=None, filter_flag=None):
        """
        Modified correct method to handle acceleration filtering
        """
        # If this is an IMU measurement (9 measurements: 3 euler, 3 angular vel, 3 accel)
        if filter_flag == "IMU":
            # Apply filters based on enabled flags
            if self.filter_euler:
                y[0:3, 0] = self.imu_filter.filter_euler(y[0:3, 0])
            if self.filter_ang_vel:
                y[3:6, 0] = self.imu_filter.filter_angular_velocity(y[3:6, 0])
            if self.filter_acc:
                y[6:9, 0] = self.imu_filter.filter_acceleration(y[6:9, 0])
        
        if filter_flag == "UWB":
            if self.filter_position:
                y[0:3, 0] = self.position_filter.filter_position(y[0:3, 0])

        I = np.eye(self.n_states)
        if np.abs(np.linalg.det(Cd @ self.P @ Cd.T + R)) > 1e-6:
            K = self.P @ Cd.T @ np.linalg.inv(Cd @ self.P @ Cd.T + R)

            if meas_model is not None:
                change_before_K = (y - meas_model(self.x[:, 0])[:, np.newaxis])
            else:
                change_before_K = (y - Cd @ self.x)

            for i in range(3):
                change_before_K[i] = ssa(change_before_K[i])
            
            change = K @ change_before_K
            
            if threshold is not None:
                for i in range(len(change)):
                    change[i, 0] = clip(change[i, 0], threshold[i])
            
            self.x = self.x + change
            self.P = (I - K @ Cd) @ self.P @ (I - K @ Cd).T + K @ R @ K.T

            for i in range(3,6):
                self.x[i] = ssa(self.x[i])
        else:
            warnings.warn("WARNING: Singular Matrix!! No correction performed")

        if self.debug:
            print("Current x (after correction):")
            print(self.x.flatten())
            
            