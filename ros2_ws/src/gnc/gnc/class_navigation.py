import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

from mav_simulator.module_kinematics import quat_to_eul, llh_to_ned, eul_to_quat, quat_to_rotm
from gnc.module_kin_kf import imu_mat, imu_model, gnss_mat, gnss_model, state_mats, plant_model

import numpy as np

class Navigation(Node):
    def __init__(self, vessel, ekf, llh0, th=None):
        super().__init__('navigation')
        self.vessel = vessel
        self.vessel_id = vessel.vessel_id
        self.vessel_name = vessel.vessel_name
        
        # Get the namespace from the node
        self.namespace = self.get_namespace()
        # self.get_logger().info(f"Node namespace: {self.namespace}")
        
        # Use the namespace for topic prefixing
        # If namespace is just '/', use empty string or a default
        if self.namespace == '/':
            self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'  # or some default
        else:
            # Remove leading '/' if present to avoid double slashes
            self.topic_prefix = self.namespace.lstrip('/')
            
        # self.get_logger().info(f"Using topic prefix: {self.topic_prefix}")
        
        self.ekf = ekf
        self.first_imu_flag = True
        self.first_pos_flag = True
        
        self.sensors = []

        for sensor in self.vessel.vessel_config['sensors']['sensors']:
            
            self.sensors.append(sensor)
            
            if sensor['sensor_type'] == 'IMU':
                if 'topic' not in sensor:
                    # Use absolute topic path with namespace
                    sensor['topic'] = f'/{self.topic_prefix}/imu/data' if self.topic_prefix else '/imu/data'
                # Create a closure to capture the current sensor
                def create_imu_callback(sensor_config):
                    return lambda msg: self.imu_callback(msg, sensor_config)
                self.imu_sub = self.create_subscription(Imu, sensor['topic'], create_imu_callback(sensor), 10)
            elif sensor['sensor_type'] == 'GPS':
                if 'topic' not in sensor:
                    sensor['topic'] = f'/{self.topic_prefix}/gps' if self.topic_prefix else '/gps'
                # Create a closure to capture the current sensor
                def create_gnss_callback(sensor_config):
                    return lambda msg: self.gnss_callback(msg, sensor_config)
                self.gnss_sub = self.create_subscription(NavSatFix, sensor['topic'], create_gnss_callback(sensor), 10)
            elif sensor['sensor_type'] == 'UWB':
                if 'topic' not in sensor:
                    sensor['topic'] = f'/{self.topic_prefix}/uwb_node/odom' if self.topic_prefix else '/uwb_node/odom'
                # Create a closure to capture the current sensor
                def create_uwb_callback(sensor_config):
                    return lambda msg: self.uwb_callback(msg, sensor_config)
                self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, sensor['topic'], create_uwb_callback(sensor), 10)
        
        self.odom_pub = self.create_publisher(Odometry, 
                                             f'/{self.topic_prefix}/odometry' if self.topic_prefix else '/odometry', 
                                             10)

        self.create_timer(self.vessel.dt, self.update_odometry)

        self.llh0 = llh0
        self.dt = self.vessel.dt
        self.th = th
    
    def imu_callback(self, msg, sensor):

        imu_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        imu_eul = quat_to_eul(imu_quat)
        imu_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_omg = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        imu_acc = quat_to_rotm(imu_quat) @ imu_acc - np.array([0, 0, -9.81])
        
        y_imu = np.concatenate((imu_eul, imu_acc, imu_omg))[:, np.newaxis]
        
        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            # self.get_logger().info(f"{sensor['custom_covariance']}")
            R_imu = np.zeros((9, 9))
            R_imu[0:3, 0:3] = np.array(sensor['custom_covariance']['orientation_covariance']).reshape(3, 3)
            R_imu[3:6, 3:6] = np.array(sensor['custom_covariance']['angular_velocity_covariance']).reshape(3, 3)
            R_imu[6:9, 6:9] = np.array(sensor['custom_covariance']['linear_acceleration_covariance']).reshape(3, 3)
        else:
            R_imu = np.zeros((9, 9))
            R_imu[0:3, 0:3] = np.array(msg.orientation_covariance).reshape(3, 3)
            R_imu[3:6, 3:6] = np.array(msg.angular_velocity_covariance).reshape(3, 3)
            R_imu[6:9, 6:9] = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
        
        Cd_imu = imu_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag or self.first_imu_flag:
            self.ekf.x[3:6] = y_imu[0:3]
            self.ekf.x[9:12] = y_imu[3:6]
            self.ekf.x[12:15] = y_imu[6:9]
            self.first_imu_flag = False
            # self.get_logger().info("First IMU Measurement Obtained")
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
        y_gnss = llh_to_ned(gnss_pos, self.llh0)[:, np.newaxis]

        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            R_gnss = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
        else:
            R_gnss = np.array(msg.position_covariance).reshape(3, 3)

        Cd_gnss = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag or self.first_imu_flag:
            self.ekf.x[0:3] = y_gnss
            self.first_pos_flag = False
            # self.get_logger().info("First GNSS Measurement Obtained")
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
        y_uwb = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])[:, np.newaxis]

        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            # self.get_logger().info(f"{sensor['custom_covariance']}")
            R_uwb = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
        else:
            R_uwb = np.zeros((3, 3))
            R_uwb[0:3, 0:3] = np.array(msg.pose.covariance).reshape(6, 6)[0:3, 0:3]

        Cd_uwb = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag or self.first_imu_flag:
            self.ekf.x[0:3] = y_uwb
            self.first_pos_flag = False
            # self.get_logger().info("First UWB Measurement Obtained")
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

        # self.get_logger().info(f"Time: {self.ekf.t:.2f}, Surge: {self.ekf.x[0,0]:.2f} m, Sway: {self.ekf.x[1,0]:.2f} m, Heave: {self.ekf.x[2,0]:.2f} m, Roll: {self.ekf.x[3,0]*180/np.pi:.2f} deg, Pitch: {self.ekf.x[4,0]*180/np.pi:.2f} deg, Yaw: {self.ekf.x[5,0]*180/np.pi:.2f} deg")

        odom_msg = Odometry()
        
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'NED'
        odom_msg.child_frame_id = 'BODY'
        
        odom_msg.pose.pose.position.x = self.ekf.x[0,0]
        odom_msg.pose.pose.position.y = self.ekf.x[1,0]
        odom_msg.pose.pose.position.z = self.ekf.x[2,0]

        quat = eul_to_quat(np.array([self.ekf.x[3,0], self.ekf.x[4,0], self.ekf.x[5,0]]))
        odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

        pose_cov = np.zeros(36).reshape(6, 6)
        pose_cov[0:3, 0:3] = self.ekf.P[0:3, 0:3]
        pose_cov[3:6, 3:6] = self.ekf.P[3:6, 3:6]
        odom_msg.pose.covariance = pose_cov.flatten()
        
        odom_msg.twist.twist.linear.x = self.ekf.x[6,0]
        odom_msg.twist.twist.linear.y = self.ekf.x[7,0]
        odom_msg.twist.twist.linear.z = self.ekf.x[8,0]
        
        odom_msg.twist.twist.angular.x = self.ekf.x[9,0]
        odom_msg.twist.twist.angular.y = self.ekf.x[10,0]
        odom_msg.twist.twist.angular.z = self.ekf.x[11,0]

        twist_cov = np.zeros(36).reshape(6, 6)
        twist_cov[0:3, 0:3] = self.ekf.P[6:9, 6:9]
        twist_cov[3:6, 3:6] = self.ekf.P[9:12, 9:12]
        odom_msg.twist.covariance = twist_cov.flatten()

        self.odom_pub.publish(odom_msg)
        
        