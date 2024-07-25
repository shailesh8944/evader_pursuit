import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
from interfaces.msg import Actuator
import module_kinematics as kin
import warnings
import scipy
from scipy.integrate import solve_ivp
import multiprocessing

class GNC():
    # This class creates a ROS2 node and initiates the callbacks for
    # IMU, GPS, UWB sensors and encoders. The class also initiates 
    # a publisher on /<vessel_name>_<vessel_id>/actuator_cmd based
    # on the guidance and control laws you will program. 

    sensor_sub = []
    odometry = {}
    actuator = {}
    rate = 10
    node = None
    topic_prefix = None
    actuator_cmd = '0.0, 0.0'

    # Non-dimensionalizing parameters
    length = None
    U_des = None

    # State estimation parameters
    x_hat = None
    P_hat = None
    u_cmd = None
    
    # Guidance parameters
    psi_des = 0.0

    # Control commands
    rudder_cmd = 0.0
    propeller_cmd = 0.0

    # Terminate flag
    terminate_flag = False

    y_p_int = 0.0
    u_err_int = 0.0
    
    current_time = 0.0
    score = 0.0

    def __init__(self, topic_prefix, 
                rate=10, 
                sensors=[], 
                gps_datum=np.array([12.99300425860631, 80.23913114094384, 94.0]),
                length=3.147,
                Fn=0.20,
                waypoint_type='XYZ',
                waypoints=None,
                vessel_data=None,
                vessel_ode=None,
                euler_angle_flag=False):
        
        self.topic_prefix = topic_prefix
        self.rate = int(rate)
        self.sensors = sensors
        self.llh0 = gps_datum
        
        self.length = length
        self.Fn = Fn
        self.U_des = self.Fn * np.sqrt(9.80665 * self.length)

        # Convert GPS waypoints to NED waypoints
        if waypoint_type == 'GPS':
            waypoints_xyz = np.zeros_like(waypoints)
            
            for i in range(waypoints.shape()[0]):
                waypoints_xyz[i, :] = kin.llh_to_ned(waypoints[i, :], self.llh0)
            
            self.waypoints = waypoints_xyz        
        else:
            self.waypoints = waypoints

        self.current_waypoint = self.waypoints[0]
        self.goal_waypoint = self.waypoints[1]
        self.waypoint_index = 0

        self.vessel_data = vessel_data
        self.vessel_ode = vessel_ode
        self.euler_angle_flag = euler_angle_flag

        if euler_angle_flag:
            self.x_hat = np.zeros(14)
            self.P_hat = np.eye(14)
            self.u_cmd = np.zeros(2)
        else:
            self.x_hat = np.zeros(15)
            self.x_hat[9] = 1.0
            self.P_hat = np.eye(15)
            self.u_cmd = np.zeros(2)
        
        self.node = Node(f'GNC_{self.topic_prefix}')
        
        # Allocate ID to each sensor
        for i in range(len(self.sensors)):
            self.sensors[i]['id'] = i

        # Create an empty measurements array
        self.sensor_measurements = [None]*len(self.sensors)

        # Register the sensors
        if len(self.sensors) != 0:
            self.register_sensors()
        
        # Register the actuator
        self.register_actuator()

        # Register odometry publisher
        self.register_odometry()
        
    def register_odometry(self):
        self.odometry['pub'] = self.node.create_publisher(Odometry, f'/{self.topic_prefix}/odometry', self.rate)
        self.odometry['timer'] = self.node.create_timer(1/self.rate, callback=self.publish_odometry)

    def register_actuator(self):
        self.actuator['pub'] = self.node.create_publisher(Actuator, f'/{self.topic_prefix}/actuator_cmd', self.rate)
        self.actuator['timer'] = self.node.create_timer(1/self.rate, callback=self.publish_actuator)

    def register_sensors(self):
        for i in range(len(self.sensors)):
            
            sensor_type = self.sensors[i]['sensor_type']
            
            if sensor_type == "IMU":

                topic_imu = self.sensors[i]['topic']
                arg_imu = i
                callback_imu = lambda x: self.imu_callback(x, arg_imu)
                h = self.node.create_subscription(Imu, topic_imu, callback_imu, 1)

            elif sensor_type == "GPS":
                
                topic_gps = self.sensors[i]['topic']
                arg_gps = i
                callback_gps = lambda x: self.gps_callback(x, arg_gps)
                h = self.node.create_subscription(NavSatFix, topic_gps, callback_gps, 1)

            elif sensor_type == "UWB":

                topic_uwb = self.sensors[i]['topic']
                arg_uwb = i
                callback_uwb = lambda x: self.uwb_callback(x, arg_uwb)
                h = self.node.create_subscription(PoseWithCovarianceStamped, topic_uwb, callback_uwb, 1)

            elif sensor_type == "encoders":

                topic_encoders = self.sensors[i]['topic']
                arg_encoders = i
                callback_encoders = lambda x: self.encoders_callback(x, arg_encoders)
                h = self.node.create_subscription(Actuator, topic_encoders, callback_encoders, 1)

            else:

                h = None
                warnings.warn(f"Sensor type {sensor_type} is unknown")

            self.sensor_sub.append(h)
    
    def imu_callback(self, msg, sensor_id):

        self.sensor_measurements[sensor_id] = msg

        q = 7; p = 2; n = 15

        y = np.zeros(q)

        y[0] = msg.angular_velocity.x / (self.U_des / self.length)
        y[1] = msg.angular_velocity.y / (self.U_des / self.length)
        y[2] = msg.angular_velocity.z / (self.U_des / self.length)
        y[3] = msg.orientation.w
        y[4] = msg.orientation.x
        y[5] = msg.orientation.y
        y[6] = msg.orientation.z
        
        q_b_i = self.sensors[sensor_id]['sensor_orientation']
        R_b_i = kin.quat_to_rotm(q_b_i)
        R_i_b = R_b_i.T

        # Measurement Noise
        Rd = np.eye(q)
        Rd[0:3][:, 0:3] = msg.angular_velocity_covariance.reshape((3, 3)) / ( (self.U_des / self.length) ** 2)
        
        Ceul = msg.orientation_covariance.reshape((3, 3))
        G = kin.dquat_deul(y[3:7])
        Cq = G @ Ceul @ G.T
        Rd[3:7][:, 3:7] = (Cq + Cq.T) / 2.0        

        # Feed forward matrix
        Dd = np.zeros((q, p))

        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR IMU MEASUREMENTS
        ################################################################################
        
        # Output matrix
        Cd = np.zeros((q, n))

        # Quaternion for rotation from IMU frame to BODY frame is available in q_b_i
        # The corresponding rotation matrix R_b_i can be calculated
        # using the function kin.quat_to_rotm(q_b_i)        
        

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)
               
    
    def uwb_callback(self, msg, sensor_id):
        
        self.sensor_measurements[sensor_id] = msg

        q = 3; p = 2; n = 15

        y = np.zeros(q)

        y[0] = msg.pose.pose.position.x
        y[1] = msg.pose.pose.position.y
        y[2] = msg.pose.pose.position.z

        y = y / self.length

        # Measurement Noise
        Rd_full = msg.pose.covariance.reshape((6, 6))
        Rd = Rd_full[0:3][:, 0:3] / (self.length ** 2)

        # Feed forward matrix
        Dd = np.zeros((q, p))
        
        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR UWB MEASUREMENTS
        ################################################################################

        # Output matrix
        Cd = np.zeros((q, n))

        # Sensor location in BODY frame
        sen_loc = self.sensors[sensor_id]['sensor_location']
        x_sen = sen_loc[0] / self.length
        y_sen = sen_loc[1] / self.length
        z_sen = sen_loc[2] / self.length

        # Quaternion for rotation from BODY frame to NED frame is available in q_n_b
        q_n_b = self.x_hat[9:13]
        qw = q_n_b[0]
        qx = q_n_b[1]
        qy = q_n_b[2]
        qz = q_n_b[3]

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)
        
    
    def encoders_callback(self, msg, sensor_id):
        
        self.sensor_measurements[sensor_id] = msg

        q = 2; p = 2; n = 15

        y = np.zeros(q)

        y[0] = msg.rudder * np.pi / 180
        y[1] = msg.propeller / 60

        y[1] = y[1] * self.length / self.U_des

        # Measurement Noise
        Rd = msg.covariance.reshape((2, 2))
        Rd[0, 1] = Rd[0, 1] / (self.U_des / self.length)
        Rd[1, 0] = Rd[1, 0] / (self.U_des / self.length)
        Rd[1, 1] = Rd[1, 1] / ((self.U_des / self.length) ** 2)

        # Feed forward matrix
        Dd = np.zeros((q, p))
        
        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR ENCODER MEASUREMENTS
        ################################################################################

        # Output matrix        
        Cd = np.zeros((q, n))        

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)
    
    def publish_actuator(self):

        current_time = self.node.get_clock().now()

        # Create Actuator message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        
        act.rudder = self.rudder_cmd * 180.0 / np.pi
        act.propeller = self.propeller_cmd * 60 * self.U_des / self.length

        self.actuator['pub'].publish(act)
    
    def publish_odometry(self):
        
        # Compute guidance and contorl
        self.guidance()
        self.control()

        # Predict EKF state estimate
        self.state_predictor()

        current_time = self.node.get_clock().now()

        # Create Actuator message
        odo = Odometry()
        odo.header.stamp = current_time.to_msg()
        odo.header.frame_id = 'NED'
        odo.child_frame_id = 'BODY'

        odo.pose.pose.position = Point(x=self.x_hat[6] * self.length, y=self.x_hat[7] * self.length, z=self.x_hat[8] * self.length)
        odo.pose.pose.orientation = Quaternion(x=self.x_hat[10], y=self.x_hat[11], z=self.x_hat[12], w=self.x_hat[9])

        odo.twist.twist.linear = Vector3(x=self.x_hat[0] * self.U_des, y=self.x_hat[1] * self.U_des, z=self.x_hat[2] * self.U_des)
        odo.twist.twist.angular = Vector3(x=self.x_hat[3] * self.U_des / self.length, y=self.x_hat[4] * self.U_des / self.length, z=self.x_hat[5] * self.U_des / self.length)

        G = kin.deul_dquat(self.x_hat[9:13])
        Cq = self.P_hat[9:13][:, 9:13]
        Ceul = G @ Cq @ G.T

        pose_cov = np.zeros((6,6), dtype=np.float64)
        pose_cov[0:3][:, 0:3] = self.P_hat[6:9][:, 6:9] * (self.length ** 2)
        pose_cov[3:6][:, 3:6] = Ceul
        odo.pose.covariance = pose_cov.flatten()

        twist_cov = np.zeros((6,6))
        twist_cov[0:3][:, 0:3] = self.P_hat[0:3][:, 0:3] * (self.U_des ** 2)
        twist_cov[3:6][:, 3:6] = self.P_hat[3:6][:, 3:6] * ((self.U_des / self.length) ** 2)
        odo.twist.covariance = twist_cov.flatten()

        self.odometry['pub'].publish(odo)

        if not self.terminate_flag:
            dt = (1 / self.rate) * self.U_des / self.length
            self.score = self.score + np.abs(self.y_p_e) * dt + np.abs(self.x_hat[13] / (35 * np.pi / 180)) * dt
            self.current_time += dt

        eul = kin.quat_to_eul(self.x_hat[9:13], deg=True)
        
        print(f"Linear Velocity (m/s)    : {self.x_hat[0] * self.U_des:.4f}, {self.x_hat[1] * self.U_des:.4f}, {self.x_hat[2] * self.U_des:.4f}")
        print(f"Angular Velocity (rad/s) : {self.x_hat[3] * self.U_des / self.length:.4f}, {self.x_hat[4] * self.U_des / self.length:.4f}, {self.x_hat[5] * self.U_des / self.length:.4f}")
        print(f"Linear Position (m)      : {self.x_hat[6] * self.length:.4f}, {self.x_hat[7] * self.length:.4f}, {self.x_hat[8] * self.length:.4f}")
        print(f"Orientation (deg)        : {eul[0]:.2f}, {eul[1]:.2f}, {eul[2]:.2f}")
        print(f"Unit quaternion          : {self.x_hat[9]:.2f}, {self.x_hat[10]:.2f}, {self.x_hat[11]:.2f}, {self.x_hat[12]:.2f}")
        print(f"Rudder angle (deg)       : {self.x_hat[13] * 180 / np.pi:.2f}")
        print(f"Propeller Speed (RPM)    : {self.x_hat[14] * 60 * self.U_des / self.length:.2f}")
        print(f"Goal Waypoint (m)        : {self.goal_waypoint[0]:.2f}, {self.goal_waypoint[1]:.2f}, {self.goal_waypoint[2]:.2f} ")
        print(f"Distance to Goal (m)     : {np.linalg.norm(self.goal_waypoint - self.x_hat[6:9] * self.length):.2f}")
        print(f"Score                    : {100 - self.score:.2f}")

        # print(f"Covariance               : {self.P_hat[0, 0]:.2f}, {self.P_hat[1,1]:.2f}, {self.P_hat[2,2]:.2f}")
        # print(f"                         : {self.P_hat[3,3]:.2f}, {self.P_hat[4,4]:.2f}, {self.P_hat[5,5]:.2f}")
        # print(f"                         : {self.P_hat[6,6]:.2f}, {self.P_hat[7,7]:.2f}, {self.P_hat[8,8]:.2f}")
        # print(f"                         : {self.P_hat[9,9]:.2f}, {self.P_hat[10,10]:.2f}, {self.P_hat[11,11]:.2f}, {self.P_hat[12,12]:.2f}")
        # print(f"                         : {self.P_hat[13,13]:.2f}, {self.P_hat[14,14]:.2f}")
        print(f'\n\n')
    
    # Extended Kalman Filter state correction based on measurements

    def state_corrector(self, y, Cd, Dd, Rd):
        n = 15
        try:
            
            #########################################################################################
            # WRITE YOUR CODE HERE TO UPDATE THE STATE AFTER MEASUREMENT
            #########################################################################################
            
            # The measurement y, output matrix Cd, feedforward matrix Dd and measurement noise Rd 
            # are inputs. 
            #
            # Available variable: 
            #
            # self.x_hat -> Current state prior to correction
            # self.P_hat -> Covariance matrix of current state prior to correction
            
            # Calculate Kalman Gain Matrix
            K = np.zeros((n, len(y)))            

            # Correction to be applied to the state based on the Kalman Gain calculated
            x_hat_corr = np.zeros(n)
            
            

            #########################################################################################

            # DO NOT EDIT THE FOLLOWING LINES (THIS IS NEEDED TO KEEP THE UPDATES STABLE)
            old_quat = self.x_hat[9:13]

            thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.6, 40])
            x_hat_corr = np.where(np.abs(x_hat_corr) > thresh, np.sign(x_hat_corr) * thresh, x_hat_corr)            
            self.x_hat = self.x_hat + x_hat_corr
            
            self.x_hat[9:13] = self.x_hat[9:13] / np.linalg.norm(self.x_hat[9:13])

            quat_diff = kin.quat_multiply(old_quat, kin.quat_conjugate(self.x_hat[9:13]))
            if quat_diff[0] < 0:
                self.x_hat[9:13] = -self.x_hat[9:13]

            # self.P_hat = (np.eye(n) - K @ Cd) @ self.P_hat @ (np.eye(n) - K @ Cd).T + K @ Rd @ K.T

        except Exception as e:
            print(e)
            pass

    # Extended Kalman Filter state prediction based on model

    def state_predictor(self):
        return          # This function has been dummied       
        n = 15; p = 2
        h = 1/self.rate * (self.U_des / self.length)

        u = self.x_hat[0]
        v = self.x_hat[1]
        r = self.x_hat[5]
        
        # Mass matrix
        M_RB = self.vessel_data['M_RB']
        M_A = self.vessel_data['M_A']
        D_l = self.vessel_data['Dl']
        K = self.vessel_data['K']
        M_inv = self.vessel_data['M_inv']

        # Cross Flow Drag coefficients
        Xuu = self.vessel_data['X_u_au']
        Yvv = self.vessel_data['Y_v_av']
        Yvr = self.vessel_data['Y_v_ar']
        Yrv = self.vessel_data['Y_r_av']
        Yrr = self.vessel_data['Y_r_ar']
        Nvv = self.vessel_data['N_v_av']
        Nvr = self.vessel_data['N_v_ar']
        Nrv = self.vessel_data['N_r_av']
        Nrr = self.vessel_data['N_r_ar']

        # Rudder and Propeller coefficients
        Yd = self.vessel_data['Y_d']
        Nd = self.vessel_data['N_d']
        # Xn = self.vessel_data['X_n']

        T_rud = self.vessel_data['T_rud']
        T_prop = self.vessel_data['T_prop']

        M = M_RB + M_A
        v_vec = self.x_hat[0:6]

        Nmat, F_damp = self.N_matrix(v_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)

        dF_damp_dv = np.zeros((6,6))
        for i in range(6):
            v2_vec = v_vec
            v1_vec = v_vec

            dx = 0.001

            v2_vec[i] = v2_vec[i] + dx
            v1_vec[i] = v1_vec[i] - dx

            _, F2_damp = self.N_matrix(v2_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)
            _, F1_damp = self.N_matrix(v1_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)

            dF_damp_dv[:, i] = (F2_damp - F1_damp) / (2 * dx)

        deul_dq = kin.deul_dquat(self.x_hat[9:13])
        H_mat = np.zeros((6, 7))
        H_mat[0:3][:, 0:3] = np.eye(3)
        H_mat[3:6][:, 3:7] = deul_dq

        Amat = np.zeros((n, n))
        Amat[0:6][:, 0:6] = - M_inv @ dF_damp_dv
        Amat[0:6][:, 6:13] = - M_inv @ K @ H_mat
        Amat[6:9][:, 0:3] = kin.quat_to_rotm(self.x_hat[9:13])
        Amat[9:13][:, 3:6] = kin.quat_rate_matrix(self.x_hat[9:13])

        wp = self.vessel_data['wp']
        tp = self.vessel_data['tp']
        D_prop = self.vessel_data['D_prop']
        pow_coeff_port = self.vessel_data['pow_coeff_port']
        pow_coeff_stbd = self.vessel_data['pow_coeff_stbd']
        
        pow_coeff = 0.5 * (pow_coeff_port + pow_coeff_stbd)

        Xn = 2 * (1 - tp) * (D_prop ** 2) * ((1 - wp) * D_prop * pow_coeff[1] + 2 * self.x_hat[14] * (D_prop ** 2) * pow_coeff[2])
        
        Amat[0:6][:, 13:15] = M_inv @ np.array([
            [0, Xn],
            [Yd, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [Nd, 0]
        ])
        
        Amat[13, 13] = -1/T_rud
        Amat[14, 14] = -1/T_prop

        Bmat = np.zeros((n, p))
        Bmat[13, 0] = 1/T_rud
        Bmat[14, 1] = 1/T_prop

        Ad = scipy.linalg.expm(Amat * h)
        
        try:
            Bd = np.linalg.inv(Ad) @ (Ad - np.eye(n)) @ Bmat
        except:
            Bd = Bmat
        
        Ed = np.zeros((n, 6))
        Ed[0:6][:, 0:6] = M_inv

        sig_process = np.array([1, 1, 1e-2, 1e-2, 1e-2, 1]) * 1e-2
        Qd = np.diag(sig_process ** 2)

        try:
            xd_hat1 = self.vessel_ode(0, self.x_hat, self.u_cmd[0], self.u_cmd[1], self.vessel_data)
            xd_hat2 = self.vessel_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat1, self.u_cmd[0], self.u_cmd[1], self.vessel_data)
            xd_hat3 = self.vessel_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat2, self.u_cmd[0], self.u_cmd[1], self.vessel_data)
            xd_hat4 = self.vessel_ode(h, self.x_hat + h * xd_hat3, self.u_cmd[0], self.u_cmd[1], self.vessel_data)

            new_state = self.x_hat + (h / 6) * (xd_hat1 + 2 * xd_hat2 + 2 * xd_hat3 + xd_hat4)

            if np.any(np.isnan(new_state)):
                raise ValueError('Predictor Diverges')
            
            old_quat = self.x_hat[9:13]

            diff = new_state - self.x_hat
            thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.6, 40])
            new_state = np.where(np.abs(diff) > thresh, self.x_hat + np.sign(diff) * thresh, new_state)
            self.x_hat = new_state
            
            self.x_hat[9:13] = self.x_hat[9:13] / np.linalg.norm(self.x_hat[9:13])
            
            quat_diff = kin.quat_multiply(old_quat, kin.quat_conjugate(self.x_hat[9:13]))
            if quat_diff[0] < 0:
                self.x_hat[9:13] = -self.x_hat[9:13]
                        
            self.P_hat = Ad @ self.P_hat @ Ad.T + Ed @ Qd @ Ed.T
            
            pass
        except Exception as e:
            # print(e)            
            pass        

    def N_matrix(self, v_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr):

            u = v_vec[0]
            v = v_vec[1]
            r = v_vec[5]

            v1 = v_vec[0:3]
            v2 = v_vec[3:6]

            D_nl = -np.array([
                [Xuu * np.abs(u), 0, 0, 0, 0, 0],
                [0, Yvv * np.abs(v) + Yvr * np.abs(r), 0, 0, 0, Yrv * np.abs(v) + Yrr * np.abs(r)],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, Nvv * np.abs(v) + Nvr * np.abs(r), 0, 0, 0, Nrv * np.abs(v) + Nrr * np.abs(r)],
            ])
            
            M11 = M[0:3][:, 0:3]
            M12 = M[0:3][:, 3:6]
            M21 = M[3:6][:, 0:3]
            M22 = M[3:6][:, 3:6]

            C = np.zeros((6, 6))
            C[0:3][:, 3:6] = -kin.Smat(M11 @ v1 + M12 @ v2)
            C[3:6][:, 0:3] = -kin.Smat(M11 @ v1 + M12 @ v2)
            C[3:6][:, 3:6] = -kin.Smat(M21 @ v1 + M22 @ v2)

            N_mat = C + D_l + D_nl

            return N_mat, N_mat @ v_vec

    def guidance(self):
        
        # Waypoint switching
        if np.linalg.norm(self.x_hat[6:9] - self.goal_waypoint / self.length) < 3 and self.terminate_flag is False:
            
            self.waypoint_index += 1

            if self.waypoint_index == self.waypoints.shape[0] - 1:
                self.terminate_flag = True

            else:                
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.goal_waypoint = self.waypoints[self.waypoint_index + 1]

        # Guidance mechanism
        pi_p = np.arctan2(self.goal_waypoint[1] - self.current_waypoint[1], 
                        self.goal_waypoint[0] - self.current_waypoint[0])

        R_n_p = np.array([[np.cos(pi_p), -np.sin(pi_p)], [np.sin(pi_p), np.cos(pi_p)]])
        
        x_n_i = self.current_waypoint[0:2] / self.length
        x_n = self.x_hat[6:8]

        xy_p_e = R_n_p.T @ (x_n - x_n_i)
        self.x_p_e = xy_p_e[0]
        self.y_p_e = xy_p_e[1]  # Cross track error

        # Look ahead distance
        Delta = 2
        kappa = 0.05

        dt = (1 / self.rate) * self.U_des / self.length

        #####################################################################################
        # WRITE YOUR CODE HERE TO DETERMINE THE DESIRED HEADING FROM ILOS GUIDANCE LAW
        #####################################################################################
        
        # Update the value of self.y_p_int (for ILOS guidance)
        yd_p_int = 0.0
        self.y_p_int = 0.0

        # Compute the desired heading angle using self.y_p_e and self.y_p_int
        self.psi_des = 0.0

        #####################################################################################

        # Calculating the u_err_int (for PI controller for propeller)
        U = np.linalg.norm(self.x_hat[0:2])
        ud_err_int = (1 - U)
        self.u_err_int = self.u_err_int + dt * ud_err_int
        

    def control(self):

        U = np.linalg.norm(self.x_hat[0:2])        
        r = self.x_hat[5]

        eul = kin.quat_to_eul(self.x_hat[9:13])
        psi = eul[2]
        
        if not self.terminate_flag:
            
            #####################################################################################
            # WRITE YOUR CODE HERE FOR THE CONTROL LAW (HINT: REMEMBER SMALLEST SIGNED ANGLE)
            #####################################################################################

            Kp_R = 0.0
            Kd_R = 0.0

            rudder_cmd = 0.0           

            #####################################################################################

            rudder_cmd = kin.clip(rudder_cmd, 35 * np.pi / 180)

            # NO NEED TO TUNE THE PROPELLER CONTROL LAW
            Kp_P = 100
            Ki_P = 10

            propeller_cmd = Kp_P * (1 - U) + Ki_P * self.u_err_int
            propeller_cmd = kin.clip(propeller_cmd, 800.0 * self.length / (self.U_des * 60.0))

            self.rudder_cmd =  rudder_cmd
            self.propeller_cmd = propeller_cmd

        else:

            self.rudder_cmd = 0.0 * np.pi / 180.0
            self.propeller_cmd = 0.0 * self.length / (self.U_des * 60.0)
        
        self.u_cmd = np.array([self.rudder_cmd, self.propeller_cmd])

