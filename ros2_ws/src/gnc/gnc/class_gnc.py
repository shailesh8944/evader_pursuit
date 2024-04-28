import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PointStamped
from interfaces.msg import Actuator
import module_kinematics as kin
import warnings

class GNC():
    sensor_sub = []
    odom_pub = None
    actuator = {}
    rate = 10
    node = None
    topic_prefix = None
    actuator_cmd = '0.0, 0.0'

    def __init__(self, topic_prefix, rate=10):
        self.topic_prefix = topic_prefix
        self.rate = rate
        self.node = Node('GNC')
        self.register_actuator()
        self.node.create_timer(1/self.rate, callback=self.publish_actuator)

    def register_actuator(self):
        self.actuator['pub'] = self.node.create_publisher(Actuator, f'/{self.topic_prefix}/actuator_cmd', self.rate)
        # self.actuator['timer'] = self.node.create_timer(1/self.rate, self.publish_actuator)


    def register_sensors(self, sensors):
        for sensor in sensors:
            if sensor['sensor_type'] == "IMU":
                h = self.node.create_subscription(Imu, sensor['topic'], self.imu_callback, 1)
            elif sensor['sensor_type'] == "GPS":
                h = self.node.create_subscription(NavSatFix, sensor['topic'], self.gps_callback, 1)
            elif sensor['sensor_type'] == "UWB":
                h = self.node.create_subscription(PointStamped, sensor['topic'], self.uwb_callback, 1)
            elif sensor['sensor_type'] == "encoders":
                h = self.node.create_subscription(Actuator, sensor['topic'], self.encoders_callback, 1)
            else:
                h = None
                warnings.warn(f"Sensor type {sensor['sensor_type']} is unknown")
            self.sensor_sub.append(h)
    
    def guidance(self):
        pass

    def control(self):
        pass
    
    def navigation(self):
        pass
    
    def imu_callback(self, msg):
        pass
    
    def gps_callback(self, msg):
        pass
    
    def uwb_callback(self, msg):
        pass
    
    def encoders_callback(self, msg):
        pass
    
    def publish_actuator(self):
        current_time = self.node.get_clock().now()

        # Create odometry message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        act.rudder = 35.0
        act.propeller = 700.0 + 100 * np.random.random()

        self.actuator['pub'].publish(act)
        self.actuator_cmd = f"{act.rudder:.2f}, {act.propeller:.2f}"
        self.node.get_logger().info(self.actuator_cmd)

