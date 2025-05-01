"""
File: class_navigation.py
Description: Defines the Navigation ROS2 node class responsible for sensor fusion using an Extended Kalman Filter (EKF). 
             It subscribes to sensor topics (IMU, GPS, UWB), processes the incoming data, 
             updates the EKF state estimate, and publishes the fused odometry information.

Author: MAV GNC Team
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

# Import utility functions for coordinate transformations and kinematic models
from mav_simulator.module_kinematics import quat_to_eul, llh_to_ned, eul_to_quat, quat_to_rotm
from gnc.module_kin_kf import imu_mat, imu_model, gnss_mat, gnss_model, state_mats, plant_model

import numpy as np

class Navigation(Node):
    """ROS2 Node for vessel state estimation using an EKF.

    Subscribes to IMU, GNSS (GPS), and potentially UWB sensor messages.
    Uses an EKF instance to fuse sensor data and predict the vessel's state 
    (position, orientation, velocity, angular rates, sensor biases).
    Publishes the estimated state as nav_msgs/Odometry messages.

    Attributes:
        vessel (object): Vessel configuration object containing parameters like ID, name, dt, sensor setup.
        ekf (EKF): Instance of the Extended Kalman Filter class.
        llh0 (np.ndarray): GPS datum (origin) in [lat, lon, h] for LLH to NED conversion.
        th (np.ndarray or None): Thresholds for EKF correction steps.
        odom_pub (rclpy.publisher.Publisher): Publisher for Odometry messages.
        imu_sub (rclpy.subscription.Subscription): Subscription to IMU messages.
        gnss_sub (rclpy.subscription.Subscription): Subscription to NavSatFix (GPS) messages.
        uwb_sub (rclpy.subscription.Subscription): Subscription to PoseWithCovarianceStamped (UWB) messages.
        first_imu_flag (bool): Flag indicating if the first IMU message has been processed.
        first_pos_flag (bool): Flag indicating if the first position message (GPS/UWB) has been processed.
    """
    def __init__(self, vessel, ekf, llh0, th=None):
        """Initializes the Navigation node.

        Args:
            vessel (object): The vessel configuration object.
            ekf (EKF): An initialized EKF object.
            llh0 (np.ndarray): The LLH datum (origin) for NED conversions.
            th (np.ndarray, optional): Thresholds for EKF corrections. Defaults to None.
        """
        # Initialize the ROS2 Node with a unique name based on vessel
        node_name = f'navigation_{vessel.vessel_name}_{vessel.vessel_id:02d}'
        super().__init__(node_name)
        self.get_logger().info(f"Initializing Navigation Node: {node_name}")
        
        self.vessel = vessel
        self.vessel_id = vessel.vessel_id
        self.vessel_name = vessel.vessel_name
        
        # Define a topic prefix for namespacing based on vessel name and ID
        self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'
            
        self.ekf = ekf # Store the provided EKF instance
        self.llh0 = llh0 # Store the GPS datum
        self.dt = self.vessel.dt # Store the time step from vessel config
        self.th = th # Store correction thresholds

        # Flags to handle initialization with the first valid sensor messages
        self.first_imu_flag = True
        self.first_pos_flag = True
        
        self.sensors = [] # Store sensor configurations

        # --- Sensor Subscription Setup --- 
        # Dynamically create subscriptions based on sensor configuration in the vessel object
        self.get_logger().info("Setting up sensor subscriptions...")
        if 'sensors' in self.vessel.vessel_config and 'sensors' in self.vessel.vessel_config['sensors']:
            for sensor in self.vessel.vessel_config['sensors']['sensors']:
                self.sensors.append(sensor)
                sensor_type = sensor['sensor_type'].upper()
                topic = sensor.get('sensor_topic', None) # Get topic or None
                
                self.get_logger().info(f"  Found sensor: {sensor_type}")
                
                if sensor_type == 'IMU':
                    if topic is None:
                        topic = f'/{self.topic_prefix}/imu/data' # Default topic name
                    # Use a lambda with captured sensor config to ensure correct context in callback
                    self.imu_sub = self.create_subscription(Imu, topic, 
                                                            lambda msg, sc=sensor: self.imu_callback(msg, sc), 
                                                            10) # QoS profile depth 10
                    self.get_logger().info(f"    Subscribing to IMU topic: {topic}")
                
                elif sensor_type == 'GPS':
                    if topic is None:
                        topic = f'/{self.topic_prefix}/gps' # Default topic name
                    self.gnss_sub = self.create_subscription(NavSatFix, topic, 
                                                             lambda msg, sc=sensor: self.gnss_callback(msg, sc), 
                                                             10)
                    self.get_logger().info(f"    Subscribing to GPS topic: {topic}")
                    
                elif sensor_type == 'UWB':
                    if topic is None:
                        topic = f'/{self.topic_prefix}/uwb' # Default topic name
                    self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, topic, 
                                                            lambda msg, sc=sensor: self.uwb_callback(msg, sc), 
                                                            10)
                    self.get_logger().info(f"    Subscribing to UWB topic: {topic}")
                else:
                    self.get_logger().warning(f"    Unsupported sensor type '{sensor_type}' defined.")
        else:
             self.get_logger().warning("No sensors defined in vessel configuration!")

        # --- Odometry Publisher Setup ---
        odom_topic = f'/{self.topic_prefix}/odometry'
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.get_logger().info(f"Publishing Odometry on: {odom_topic}")

        # --- EKF Prediction Timer ---
        # Create a timer that triggers the EKF prediction step periodically
        self.create_timer(self.dt, self.update_odometry)
        self.get_logger().info(f"EKF prediction timer started with dt = {self.dt}s")
    
    def imu_callback(self, msg: Imu, sensor: dict):
        """Callback function for processing IMU messages."""
        # Extract data from IMU message
        imu_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        imu_eul = quat_to_eul(imu_quat) # Convert quaternion to Euler angles (roll, pitch, yaw)
        imu_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_omg = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        # Transform acceleration to a different frame (potentially removing gravity) 
        # Assumes IMU measures specific force (accel - gravity). Rotation corrects frame, then subtracts gravity vector.
        # NOTE: This assumes the IMU frame is aligned with the body frame after rotation. Check conventions.
        # NOTE: Assumes standard gravity [0, 0, -9.81] in the target frame (likely NED or ENU).
        try:
            imu_acc = quat_to_rotm(imu_quat) @ imu_acc - np.array([0, 0, -9.81]) 
        except Exception as e:
            self.get_logger().error(f"Error transforming IMU acceleration: {e}")
            return

        # Construct the measurement vector y = [roll, pitch, yaw, acc_x, acc_y, acc_z, omg_x, omg_y, omg_z]
        # Order seems different from typical EKF state order. Check imu_model/imu_mat consistency.
        y_imu = np.concatenate((imu_eul, imu_acc, imu_omg))[:, np.newaxis]
        
        # Sensor frame offset (position and orientation relative to body frame)
        r_bs_b = np.array(sensor.get('sensor_location', [0.0, 0.0, 0.0]))
        Theta_bs = np.array(sensor.get('sensor_orientation', [0.0, 0.0, 0.0]))

        # Get measurement noise covariance matrix (R)
        if sensor.get('use_custom_covariance', False):
            try:
                # Use covariance specified in the vessel config file
                R_imu = np.zeros((9, 9))
                # Note: IMU msg covariance order: orientation, ang_vel, lin_acc.
                # Code below maps config keys to R matrix blocks.
                R_imu[0:3, 0:3] = np.array(sensor['custom_covariance']['orientation_covariance']).reshape(3, 3)
                R_imu[3:6, 3:6] = np.array(sensor['custom_covariance']['angular_velocity_covariance']).reshape(3, 3)
                R_imu[6:9, 6:9] = np.array(sensor['custom_covariance']['linear_acceleration_covariance']).reshape(3, 3)
            except KeyError as e:
                 self.get_logger().error(f"Missing key in custom IMU covariance: {e}")
                 return
            except Exception as e:
                 self.get_logger().error(f"Error processing custom IMU covariance: {e}")
                 return
        else:
            # Use covariance from the IMU message itself
            R_imu = np.zeros((9, 9))
            try:
                R_imu[0:3, 0:3] = np.array(msg.orientation_covariance).reshape(3, 3)
                R_imu[3:6, 3:6] = np.array(msg.angular_velocity_covariance).reshape(3, 3)
                R_imu[6:9, 6:9] = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
            except Exception as e:
                 self.get_logger().error(f"Error processing message IMU covariance: {e}")
                 return
        
        # Calculate the measurement matrix (Cd) or Jacobian for IMU
        Cd_imu = imu_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)
        
        # Handle first message(s) for initialization
        if self.first_pos_flag or self.first_imu_flag:
            # Initialize EKF state components using the first IMU reading
            # This assumes the first reading is reasonably accurate for attitude and rates/accels.
            # Indices correspond to EKF state: [pos(3), att(3), vel(3), bias_acc(3), bias_gyro(3)]? Check EKF state definition.
            # Mapping y_imu indices to EKF state indices seems inconsistent here.
            # y_imu[0:3] = eul, y_imu[3:6] = acc, y_imu[6:9] = omg 
            # ekf.x[3:6] = att, ekf.x[9:12] = bias_acc?, ekf.x[12:15] = bias_gyro?
            # **CRITICAL**: Review EKF state order and this initialization logic.
            self.get_logger().warning("Initializing EKF attitude/bias from first IMU message. Verify state mapping.")
            self.ekf.x[3:6] = y_imu[0:3] # EKF attitude = IMU Euler
            # Assuming EKF state 9:12 is acc_bias and 12:15 is gyro_bias
            # Initialize biases to zero or potentially use IMU reading? Setting biases from accel/rate is unusual.
            # self.ekf.x[9:12] = y_imu[3:6] # EKF acc_bias = IMU accel? Seems wrong.
            # self.ekf.x[12:15] = y_imu[6:9] # EKF gyro_bias = IMU gyro rate? Seems wrong.
            # Perhaps initialize biases to zero instead:
            # self.ekf.x[9:12] = np.zeros((3,1))
            # self.ekf.x[12:15] = np.zeros((3,1))

            if self.first_imu_flag:
                self.get_logger().info("First IMU measurement processed for initialization.")
                self.first_imu_flag = False
        else:
            # Perform EKF correction step
            try:
                self.ekf.correct(
                    y_imu, 
                    Cd_imu, 
                    R_imu, 
                    meas_model=lambda x: imu_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), # Measurement function h(x)
                    threshold=self.th, # Correction thresholds
                    imu_ssa=True # Apply smallest signed angle to attitude errors
                )
            except Exception as e:
                self.get_logger().error(f"EKF correction failed for IMU data: {e}")
            
    def gnss_callback(self, msg: NavSatFix, sensor: dict):
        """Callback function for processing GNSS (GPS) messages."""
        # Extract LLH position from message
        gnss_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        # Convert LLH to NED frame using the provided datum
        try:
            y_gnss = llh_to_ned(gnss_pos, self.llh0)[:, np.newaxis] # Ensure column vector
        except Exception as e:
            self.get_logger().error(f"LLH to NED conversion failed: {e}")
            return

        # Sensor frame offset (position and orientation relative to body frame)
        r_bs_b = np.array(sensor.get('sensor_location', [0.0, 0.0, 0.0]))
        Theta_bs = np.array(sensor.get('sensor_orientation', [0.0, 0.0, 0.0]))

        # Get measurement noise covariance matrix (R)
        if sensor.get('use_custom_covariance', False):
            try:
                # Use covariance specified in the vessel config file
                R_gnss = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
            except KeyError as e:
                 self.get_logger().error(f"Missing key in custom GNSS covariance: {e}")
                 return
            except Exception as e:
                 self.get_logger().error(f"Error processing custom GNSS covariance: {e}")
                 return
        else:
            # Use covariance from the NavSatFix message itself
            try:
                # Note: NavSatFix covariance is 3x3, stored row-major.
                R_gnss = np.array(msg.position_covariance).reshape(3, 3)
            except Exception as e:
                 self.get_logger().error(f"Error processing message GNSS covariance: {e}")
                 return

        # Calculate the measurement matrix (Cd) or Jacobian for GNSS
        Cd_gnss = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        # Handle first message(s) for initialization
        if self.first_pos_flag or self.first_imu_flag:
            # Initialize EKF position state with the first GNSS reading
            self.ekf.x[0:3] = y_gnss
            if self.first_pos_flag:
                self.get_logger().info(f"First GNSS measurement processed. Initializing EKF position: {y_gnss.flatten()}")
                self.first_pos_flag = False
        else:
            # Perform EKF correction step
            try:
                self.ekf.correct(
                    y_gnss, 
                    Cd_gnss, 
                    R_gnss, 
                    meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), # Measurement function h(x)
                    threshold=self.th, # Correction thresholds
                    imu_ssa=False # Do not apply SSA for position errors
                )
            except Exception as e:
                 self.get_logger().error(f"EKF correction failed for GNSS data: {e}")
        
    def uwb_callback(self, msg: PoseWithCovarianceStamped, sensor: dict):
        """Callback function for processing UWB position messages."""
        # Extract position from UWB message (assuming NED or compatible frame)
        # Ensure it's a column vector
        y_uwb = np.array([msg.pose.pose.position.x, 
                          msg.pose.pose.position.y, 
                          msg.pose.pose.position.z])[:, np.newaxis]

        # Sensor frame offset (position and orientation relative to body frame)
        r_bs_b = np.array(sensor.get('sensor_location', [0.0, 0.0, 0.0]))
        Theta_bs = np.array(sensor.get('sensor_orientation', [0.0, 0.0, 0.0]))

        # Get measurement noise covariance matrix (R)
        if sensor.get('use_custom_covariance', False):
            try:
                 # Use covariance specified in the vessel config file
                R_uwb = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
            except KeyError as e:
                 self.get_logger().error(f"Missing key in custom UWB covariance: {e}")
                 return
            except Exception as e:
                 self.get_logger().error(f"Error processing custom UWB covariance: {e}")
                 return
        else:
            # Use covariance from the PoseWithCovarianceStamped message
            # Note: Covariance in message is 6x6 (pose). We only need the 3x3 position part.
            try:
                full_cov = np.array(msg.pose.covariance).reshape(6, 6)
                R_uwb = full_cov[0:3, 0:3]
            except Exception as e:
                 self.get_logger().error(f"Error processing message UWB covariance: {e}")
                 return
        
        # Calculate the measurement matrix (Cd) or Jacobian for UWB (using gnss_mat as it's position)
        Cd_uwb = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        # Handle first message(s) for initialization
        if self.first_pos_flag or self.first_imu_flag:
            # Initialize EKF position state with the first UWB reading
            self.ekf.x[0:3] = y_uwb
            if self.first_pos_flag:
                self.get_logger().info(f"First UWB measurement processed. Initializing EKF position: {y_uwb.flatten()}")
                self.first_pos_flag = False
        else:
            # Perform EKF correction step
            try:
                self.ekf.correct(
                    y_uwb, 
                    Cd_uwb, 
                    R_uwb, 
                    meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), # Measurement function h(x), re-using GNSS model
                    threshold=self.th, # Correction thresholds
                    imu_ssa=False # Do not apply SSA for position errors
                )
            except Exception as e:
                 self.get_logger().error(f"EKF correction failed for UWB data: {e}")
           
    def update_odometry(self):
        """Performs the EKF prediction step and publishes the estimated Odometry."""
        
        # --- EKF Prediction Step ---
        try:
            # Calculate state transition matrix (A) and process noise matrix (E) based on current state
            Amat, Emat = state_mats(self.ekf.x.flatten()) # Requires flattened state
            
            # Perform the prediction
            # u (control input) is assumed zero here. If control inputs affect kinematics, pass them.
            self.ekf.predict(
                u=np.array([[0]]), # Control input vector (currently unused)
                A=Amat,             # State transition matrix (or Jacobian F)
                B=None,             # Control input matrix (or Jacobian G)
                E=Emat,             # Process noise input matrix
                plant_model=plant_model, # Non-linear state transition function f(x, u)
                discrete_flag=False, # Indicates continuous-time model used internally by EKF?
                threshold=self.th  # Thresholds (purpose in predict step is unclear, check EKF implementation)
            )
        except Exception as e:
            self.get_logger().error(f"EKF prediction failed: {e}")
            return # Skip publishing if prediction fails

        # Optional logging of the predicted state
        # self.get_logger().info(f"Time: {self.ekf.t:.2f}, Pos: {self.ekf.x[0:3,0]}, Att: {self.ekf.x[3:6,0]*180/np.pi}")

        # --- Publish Odometry Message ---
        odom_msg = Odometry()
        
        # Set header information
        odom_msg.header.stamp = self.get_clock().now().to_msg() # Current ROS time
        odom_msg.header.frame_id = 'NED' # Reference frame (e.g., 'odom', 'map', 'NED')
        odom_msg.child_frame_id = 'BODY' # Frame of the vessel itself (e.g., 'base_link', 'BODY')
        
        # Populate pose information (position and orientation)
        # Assuming EKF state: [pos(3), att(3), vel(3), ang_vel(3/bias?), bias(6?)]
        # **CRITICAL**: Confirm EKF state vector order [pos, att, vel, ang_vel, bias_acc, bias_gyro?]
        try:
            # Position (indices 0, 1, 2)
            odom_msg.pose.pose.position.x = self.ekf.x[0,0]
            odom_msg.pose.pose.position.y = self.ekf.x[1,0]
            odom_msg.pose.pose.position.z = self.ekf.x[2,0]

            # Orientation (indices 3, 4, 5 - assuming Euler angles)
            eul_angles = np.array([self.ekf.x[3,0], self.ekf.x[4,0], self.ekf.x[5,0]])
            quat = eul_to_quat(eul_angles) # Convert Euler to Quaternion for ROS message
            odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

            # Populate pose covariance matrix (6x6)
            pose_cov = np.zeros((6, 6))
            pose_cov[0:3, 0:3] = self.ekf.P[0:3, 0:3] # Position covariance
            pose_cov[3:6, 3:6] = self.ekf.P[3:6, 3:6] # Orientation covariance
            odom_msg.pose.covariance = pose_cov.flatten().tolist() # Flatten and convert to list
            
            # Populate twist information (linear and angular velocities)
            # Linear velocity (indices 6, 7, 8)
            odom_msg.twist.twist.linear.x = self.ekf.x[6,0]
            odom_msg.twist.twist.linear.y = self.ekf.x[7,0]
            odom_msg.twist.twist.linear.z = self.ekf.x[8,0]
            
            # Angular velocity (indices 9, 10, 11 - Body frame?)
            odom_msg.twist.twist.angular.x = self.ekf.x[9,0]
            odom_msg.twist.twist.angular.y = self.ekf.x[10,0]
            odom_msg.twist.twist.angular.z = self.ekf.x[11,0]

            # Populate twist covariance matrix (6x6)
            twist_cov = np.zeros((6, 6))
            twist_cov[0:3, 0:3] = self.ekf.P[6:9, 6:9]   # Linear velocity covariance
            twist_cov[3:6, 3:6] = self.ekf.P[9:12, 9:12] # Angular velocity covariance
            odom_msg.twist.covariance = twist_cov.flatten().tolist() # Flatten and convert to list

            # Publish the message
            self.odom_pub.publish(odom_msg)
            
        except IndexError:
            self.get_logger().error("IndexError while populating Odometry message. EKF state size or order mismatch?")
        except Exception as e:
             self.get_logger().error(f"Error creating or publishing Odometry message: {e}")
        