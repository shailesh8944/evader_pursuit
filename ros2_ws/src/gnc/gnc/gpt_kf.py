import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # Initialize state and covariance
        self.dt = 0.1  # Time step (adjust as needed)
        self.state = np.zeros(12)  # [x, y, z, phi, theta, psi, vx, vy, vz, wx, wy, wz]
        self.P = np.eye(12) * 0.1  # Initial covariance

        # Define process noise and measurement noise
        self.Q = np.eye(12) * 0.01  # Process noise covariance
        self.R_imu = np.eye(6) * 0.1  # Measurement noise for IMU (acc, gyro)
        self.R_gps = np.eye(3) * 5.0  # Measurement noise for GPS (position)

        # Subscribe to IMU and GPS data
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)

        # Publisher for the fused odometry
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)

    def predict(self):
        # State prediction model
        F = np.eye(12)  # State transition matrix
        F[0, 6] = F[1, 7] = F[2, 8] = self.dt  # Position update with velocity
        F[3, 9] = F[4, 10] = F[5, 11] = self.dt  # Orientation update with angular velocity

        # Predict the next state and covariance
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

    def update_imu(self, imu_data):
        # IMU measurement update
        z = np.array([
            imu_data.linear_acceleration.x,
            imu_data.linear_acceleration.y,
            imu_data.linear_acceleration.z,
            imu_data.angular_velocity.x,
            imu_data.angular_velocity.y,
            imu_data.angular_velocity.z
        ])

        H = np.zeros((6, 12))  # Measurement matrix for IMU
        H[0, 6] = H[1, 7] = H[2, 8] = 1  # Map acceleration to velocities
        H[3, 9] = H[4, 10] = H[5, 11] = 1  # Map angular velocities

        # Kalman gain
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        y = z - H @ self.state  # Measurement residual
        self.state = self.state + K @ y
        self.P = (np.eye(12) - K @ H) @ self.P

    def update_gps(self, gps_data):
        # GPS measurement update
        z = np.array([gps_data.latitude, gps_data.longitude, gps_data.altitude])

        # Convert GPS to local frame (simple approach, refine for accurate results)
        # Assuming local frame is around the initial GPS coordinates
        if not hasattr(self, 'gps_initial'):
            self.gps_initial = z  # Initialize GPS reference frame

        # Convert GPS coordinates to local frame (meters)
        z_local = np.array([
            (z[0] - self.gps_initial[0]) * 111139,  # Latitude to meters
            (z[1] - self.gps_initial[1]) * 111139 * np.cos(np.radians(self.gps_initial[0])),  # Longitude to meters
            z[2] - self.gps_initial[2]  # Altitude in meters
        ])

        H = np.zeros((3, 12))  # Measurement matrix for GPS
        H[0, 0] = H[1, 1] = H[2, 2] = 1  # Map positions

        # Kalman gain
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        y = z_local - H @ self.state  # Measurement residual
        self.state = self.state + K @ y
        self.P = (np.eye(12) - K @ H) @ self.P

    def imu_callback(self, msg: Imu):
        self.predict()  # Predict the state based on the motion model
        self.update_imu(msg)  # Update with IMU data
        self.publish_odometry()

    def gps_callback(self, msg: NavSatFix):
        self.update_gps(msg)  # Update with GPS data
        self.publish_odometry()

    def publish_odometry(self):
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Fill in position
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = self.state[2]

        # Fill in orientation (Euler angles to quaternion)
        q = self.euler_to_quaternion(self.state[3], self.state[4], self.state[5])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Fill in velocity
        odom_msg.twist.twist.linear.x = self.state[6]
        odom_msg.twist.twist.linear.y = self.state[7]
        odom_msg.twist.twist.linear.z = self.state[8]
        odom_msg.twist.twist.angular.x = self.state[9]
        odom_msg.twist.twist.angular.y = self.state[10]
        odom_msg.twist.twist.angular.z = self.state[11]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        # Convert Euler angles to quaternion
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
