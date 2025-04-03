import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped

from mav_simulator.module_kinematics import quat_to_eul, llh_to_ned
from gnc.module_kin_kf import imu_mat, imu_model, gnss_mat, gnss_model, state_mats, plant_model

import numpy as np

class Navigation(Node):
    def __init__(self, vessel, ekf, llh0):
        super().__init__('navigation')
        self.vessel = vessel
        self.ekf = ekf
        self.first_imu_flag = True
        self.first_pos_flag = True

        for sensor in self.vessel.vessel_config['sensors']['sensors']:            
            if sensor['sensor_type'] == 'IMU':
                self.imu_sub = self.create_subscription(Imu, sensor['topic'], lambda msg: self.imu_callback(msg, sensor), 10)
            elif sensor['sensor_type'] == 'GPS':
                self.gnss_sub = self.create_subscription(NavSatFix, sensor['topic'], lambda msg: self.gnss_callback(msg, sensor), 10)
            elif sensor['sensor_type'] == 'UWB':
                self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, sensor['topic'], lambda msg: self.uwb_callback(msg, sensor), 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)

        self.create_timer(self.vessel.dt, self.update_odometry)

        self.llh0 = llh0
        self.dt = self.vessel.dt
        self.th = None

    def calculate_threshold(self, dt):
        # Threshold for correction
        th = np.full(15, np.inf)
                
        # Thresholds for positions
        th[0] = np.inf * dt
        th[1] = np.inf * dt
        th[2] = 1 / 1000000

        th[6:9] = th[0:3] * dt
        th[12:15] = th[6:9] * dt

        return th
    
    def imu_callback(self, msg, sensor):

        imu_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        imu_eul = quat_to_eul(imu_quat)
        imu_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_omg = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        y_imu = np.concatenate((imu_eul, imu_acc, imu_omg))[:, np.newaxis]
        
        r_bs_b = np.array(sensor['location'])
        Theta_bs = np.array(sensor['orientation'])
        
        if sensor['user_covariance']:
            R_imu = np.array(sensor['covariance']).reshape(9, 9)
        else:
            R_imu = np.zeros((9, 9))
            R_imu[0:3, 0:3] = np.array(msg.orientation_covariance).reshape(3, 3)
            R_imu[3:6, 3:6] = np.array(msg.angular_velocity_covariance).reshape(3, 3)
            R_imu[6:9, 6:9] = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
        
        Cd_imu = imu_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_imu_flag:
            self.ekf.x[3:6, 0] = y_imu[0:3]
            self.ekf.x[9:12, 0] = y_imu[3:6]
            self.ekf.x[12:15, 0] = y_imu[6:9]
            self.first_imu_flag = False
            self.get_logger().info("First IMU Measurement Obtained")
        else:
            self.ekf.correct(
                y_imu, 
                Cd_imu, 
                R_imu, 
                meas_model=lambda x: imu_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th,
                imu_ssa=True
            )        
        
    def gnss_callback(self, msg, sensor):
        
        gnss_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        y_gnss = llh_to_ned(gnss_pos, self.llh0)

        r_bs_b = np.array(sensor['location'])
        Theta_bs = np.array(sensor['orientation'])

        if sensor['user_covariance']:
            R_gnss = np.array(sensor['covariance']).reshape(3, 3)
        else:
            R_gnss = np.array(msg.position_covariance).reshape(3, 3)

        Cd_gnss = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag:
            self.ekf.x[0:3, 0] = y_gnss
            self.first_pos_flag = False
            self.get_logger().info("First GNSS Measurement Obtained")
        else:
            self.ekf.correct(
                y_gnss, 
                Cd_gnss, 
                R_gnss, 
                meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th,
                imu_ssa=False
            )
        
    def uwb_callback(self, msg, sensor):
        y_uwb = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        r_bs_b = np.array(sensor['location'])
        Theta_bs = np.array(sensor['orientation'])
        
        if sensor['user_covariance']:
            R_uwb = np.array(sensor['covariance']).reshape(3, 3)
        else:
            R_uwb = np.zeros((3, 3))
            R_uwb[0:3, 0:3] = np.array(msg.pose.covariance).reshape(6, 6)[0:3, 0:3]

        Cd_uwb = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag:
            self.ekf.x[0:3, 0] = y_uwb
            self.first_pos_flag = False
            self.get_logger().info("First UWB Measurement Obtained")
        else:
            self.ekf.correct(
                y_uwb, 
                Cd_uwb, 
                R_uwb, 
                meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th
            )
        
    def update_odometry(self):

        Amat, Emat = state_mats(self.ekf.x.flatten())
        self.ekf.predict(
            u=np.array([[0]]), 
            A=Amat, 
            B=None, 
            E=Emat,
            plant_model=plant_model,
            discrete_flag=False,
            threshold=self.th
        )

        self.get_logger().info(f"Time: {self.ekf.t:.2f}, Surge: {self.ekf.x[0,0]:.2f} m, Sway: {self.ekf.x[1,0]:.2f} m, Heave: {self.ekf.x[2,0]:.2f} m, Roll: {self.ekf.x[3,0]*180/np.pi:.2f} deg, Pitch: {self.ekf.x[4,0]*180/np.pi:.2f} deg, Yaw: {self.ekf.x[5,0]*180/np.pi:.2f} deg")
        

        
            
        
        
        