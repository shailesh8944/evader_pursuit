from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from geometry_msgs.msg import PoseArray, Pose
from mav_simulator.module_kinematics import quat_to_eul
import gnc.module_control as con
import numpy as np

class GuidanceControl(Node):
    def __init__(self, vessel):
        super().__init__('guidance_controller')

        self.vessel = vessel
        # self.odom_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/odometry'
        self.odom_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/odometry_sim'
        self.actuator_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/actuator_cmd'
        self.waypoints_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/waypoints'
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
            
        # Create publisher for rudder command
        self.actuator_pub = self.create_publisher(
            Actuator,
            self.actuator_topic,
            10)
        
        # Create publisher for waypoints
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            self.waypoints_topic,
            10)
        
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second
        
        # Read waypoints
        self.waypoints = np.array(self.vessel.vessel_config['guidance']['waypoints'])
        self.waypoints_type = self.vessel.vessel_config['guidance']['waypoints_type']
        self.waypoint_idx = 1

        self.waypoints_data = {
            'waypoints': self.waypoints,
            'waypoints_type': self.waypoints_type
        }
        
        # Initialize time
        self.start_time = self.get_clock().now()

        # Initialize integrator
        self.ye_int = 0.0
        self.te_int = 0.0
        
        # Select control mode
        self.control_mode = con.pid_control
        
        self.get_logger().info('Guidance Controller initialized')

    def publish_waypoints(self):
        if not self.waypoints_data:
            return
            
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        for waypoint in self.waypoints_data.get('waypoints', []):
            pose = Pose()
            pose.position.x = float(waypoint[0])
            pose.position.y = float(waypoint[1])
            pose.position.z = float(waypoint[2])
            pose_array.poses.append(pose)
        
        self.waypoints_pub.publish(pose_array)
    
    def odom_callback(self, msg):
        # Get current time in seconds
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9

        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        eul = quat_to_eul(quat, order='ZYX')
        
        # Create state vector from odometry
        state = np.array([            
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            eul[0],
            eul[1],
            eul[2],
            0.0 # rudder angle
        ])

        # Calculate control command
        rudder_cmd, ye, wp_indx = self.control_mode(t, state, self.waypoints, self.waypoint_idx, self.ye_int)

        if wp_indx != self.waypoint_idx:
            self.get_logger().info(f'Waypoint {self.waypoint_idx} reached')
            self.waypoint_idx = wp_indx
            self.ye_int = 0.0

            if self.waypoint_idx >= len(self.waypoints):
                self.get_logger().info('***All waypoints reached***')
                self.waypoint_idx = 1

        # Calculate time difference for integration
        dt = t - self.te_int
        if dt > 0:  # Avoid division by zero or negative time
            # Update the integral term
            self.ye_int += ye * dt
            self.te_int = t
        
        # Publish command
        cmd_msg = Actuator()
        cmd_msg.actuator_values = [float(rudder_cmd * 180 / np.pi)]
        cmd_msg.actuator_names = ['cs_1']
        cmd_msg.covariance = [0.0]
        self.actuator_pub.publish(cmd_msg)
        
        # Log info
        # self.get_logger().info(
        #     f'\nTime: {t:.2f}s\n'            
        #     f'Rudder Command: {rudder_cmd*180/np.pi:.2f} deg\n'
        # )