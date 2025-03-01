import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
# from mav_simulator.interfaces.msg import Actuator
import module_kinematics as kin
from module_sensors import create_sensor

class Vessel_Pub_Sub():
    """ROS2 publisher/subscriber interface for a vessel.
    
    This class handles all ROS2 communication for a vessel, including:
    - Publishing sensor data
    - Publishing odometry
    - Subscribing to actuator commands
    """
    
    def __init__(self, vessel, world_node):
        """Initialize the vessel interface.
        
        Args:
            vessel: The vessel object this interface is for
            node: The ROS2 node to use for communication
        """
        self.world_node = world_node
        self.vessel = vessel
        self.vessel_id = vessel.vessel_id
        self.vessel_name = vessel.vessel_name
        self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'
        
        # Initialize commanded values for all actuators
        self.delta_c = np.zeros(len(vessel.vessel_config.get('control_surfaces', {}).get('control_surfaces', [])))
        self.n_c = np.zeros(len(vessel.vessel_config.get('thrusters', {}).get('thrusters', [])))
        

        # Create sensors
        self.sensors = []
        if 'sensors' in vessel.vessel_config:
            for sensor_config in vessel.vessel_config['sensors'].get('sensors', []):
                # Update sensor topic to use {topic_prefix}/{sensor_name} format
                sensor_name = sensor_config.get('name', sensor_config['sensor_type'].lower())
                sensor_topic = f'{self.topic_prefix}/{sensor_name}'
                
                # Update sensor_config with new topic
                sensor_config['topic'] = sensor_topic
                
                sensor = create_sensor(sensor_config, self.vessel_id, self.topic_prefix, self)
                self.sensors.append({
                    'sensor': sensor,
                    'pub': self.world_node.create_publisher(self._get_msg_type(sensor.sensor_type), sensor_topic, 10),
                    'timer': self.world_node.create_timer(1/sensor.rate, lambda s=sensor: self._publish_sensor(s))
                })
        
        # Create odometry publisher
        self.odometry = {
            'pub': self.world_node.create_publisher(Odometry, f'{self.topic_prefix}/odometry_sim', 10),
            'timer': self.world_node.create_timer(self.vessel.vessel_config['time_step'], self.publish_odometry)
        }
        
        # # Create actuator subscribers
        # self.actuator_sub = self.create_subscription(
        #     Actuator, 
        #     f'{self.topic_prefix}/actuator_cmd', 
        #     self.actuator_callback, 
        #     1
        # )

    def _get_msg_type(self, sensor_type):
        """Get the ROS message type for a given sensor type."""
        msg_types = {
            'IMU': Imu,
            'GPS': NavSatFix,
            'UWB': PoseWithCovarianceStamped,
            # 'encoders': Actuator
        }
        return msg_types.get(sensor_type)

    def _publish_sensor(self, sensor):
        """Publish sensor data.
        
        Args:
            sensor: The sensor object to publish data for
        """
        measurement = sensor.get_measurement()
        msg = self._create_sensor_message(sensor.sensor_type, measurement)
        
        # Find the publisher for this sensor
        for s in self.sensors:
            if s['sensor'] == sensor:
                s['pub'].publish(msg)
                break

    def _create_sensor_message(self, sensor_type, measurement):
        """Create a ROS message for sensor data.
        
        Args:
            sensor_type: Type of the sensor
            measurement: Dictionary containing sensor measurements
            
        Returns:
            The appropriate ROS message type filled with sensor data
        """
        current_time = self.world_node.get_clock().now().to_msg()
        
        if sensor_type == 'IMU':
            msg = Imu()
            msg.header.stamp = current_time
            msg.header.frame_id = f'{self.topic_prefix}_imu_frame'
            msg.orientation = Quaternion(
                x=measurement['orientation'][1],
                y=measurement['orientation'][2],
                z=measurement['orientation'][3],
                w=measurement['orientation'][0]
            )
            msg.angular_velocity = Vector3(
                x=measurement['angular_velocity'][0],
                y=measurement['angular_velocity'][1],
                z=measurement['angular_velocity'][2]
            )
            msg.linear_acceleration = Vector3(
                x=measurement['linear_acceleration'][0],
                y=measurement['linear_acceleration'][1],
                z=measurement['linear_acceleration'][2]
            )
            msg.orientation_covariance = measurement['orientation_covariance']
            msg.angular_velocity_covariance = measurement['angular_velocity_covariance']
            msg.linear_acceleration_covariance = measurement['linear_acceleration_covariance']
            
        elif sensor_type == 'GPS':
            msg = NavSatFix()
            msg.header.stamp = current_time
            msg.header.frame_id = 'ECEF'
            msg.status = NavSatStatus(status=0, service=1)
            msg.latitude = measurement['latitude']
            msg.longitude = measurement['longitude']
            msg.altitude = measurement['altitude']
            msg.position_covariance = measurement['position_covariance']
            
        elif sensor_type == 'UWB':
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = current_time
            msg.header.frame_id = 'NED'
            msg.pose.pose.position = Point(
                x=measurement['position'][0],
                y=measurement['position'][1],
                z=measurement['position'][2]
            )
            msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.pose.covariance = measurement['covariance']
            
        # elif sensor_type == 'encoders':
        #     msg = Actuator()
        #     msg.header.stamp = current_time
        #     msg.rudder = measurement['rudder']
        #     msg.propeller = measurement['propeller']
        #     msg.covariance = measurement['covariance']
            
        return msg

    def actuator_callback(self, msg):
        """Handle actuator command messages."""
        # Update commanded values for all actuators
        if len(self.delta_c) > 0:
            self.delta_c[0] = msg.rudder * np.pi / 180.0  # Convert to radians
        if len(self.n_c) > 0:
            self.n_c[0] = msg.propeller / 60.0  # Convert to RPS

    def publish_odometry(self):
        """Publish vessel odometry data."""
        current_time = self.world_node.get_clock().now()
        
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'NED'
        msg.child_frame_id = 'BODY'
        
        state = self.vessel.current_state
        
        # Position and orientation
        msg.pose.pose.position = Point(x=state[6], y=state[7], z=state[8])
        msg.pose.pose.orientation = Quaternion(
            x=state[10], y=state[11], z=state[12], w=state[9]
        )
        
        # Linear and angular velocities
        msg.twist.twist.linear = Vector3(x=state[0], y=state[1], z=state[2])
        msg.twist.twist.angular = Vector3(x=state[3], y=state[4], z=state[5])
        
        # Zero covariance as this is simulation data
        msg.pose.covariance = np.zeros((6, 6)).flatten()
        msg.twist.covariance = np.zeros((6, 6)).flatten()
        
        self.odometry['pub'].publish(msg)