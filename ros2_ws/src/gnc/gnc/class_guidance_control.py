"""
File: class_guidance_control.py
Description: Implements the GuidanceControl ROS2 node. This node subscribes to vessel odometry,
             retrieves waypoints from the vessel configuration, calculates rudder commands 
             using a specified control law (e.g., PID/LOS from module_control), 
             and publishes these commands to an actuator topic. It also visualizes the waypoints.

Author: MAV GNC Team
"""

from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator # Custom message type for actuator commands
from geometry_msgs.msg import PoseArray, Pose # For publishing waypoints for visualization
from mav_simulator.module_kinematics import quat_to_eul # Utility for quaternion to Euler conversion
import gnc.module_control as con # Imports the control algorithms (e.g., pid_control)
import numpy as np

class GuidanceControl(Node):
    """ROS2 Node for vessel guidance and control.

    Listens to odometry, determines the required rudder angle to follow a 
    predefined set of waypoints using a selected control algorithm, and 
    publishes the command.

    Attributes:
        vessel (object): Vessel configuration object.
        odom_sub (rclpy.subscription.Subscription): Subscriber to Odometry messages.
        actuator_pub (rclpy.publisher.Publisher): Publisher for Actuator command messages.
        waypoints_pub (rclpy.publisher.Publisher): Publisher for PoseArray waypoint visualization.
        waypoints (np.ndarray): Array of waypoints [[x1, y1, z1], [x2, y2, z2], ...].
        waypoint_idx (int): Index of the current target waypoint.
        ye_int (float): Integral of the cross-track error (used by some controllers).
        te_int (float): Timestamp of the last integration step for ye_int.
        control_mode (callable): Function handle to the selected control law (e.g., con.pid_control).
    """
    def __init__(self, vessel):
        """Initializes the GuidanceControl node.

        Args:
            vessel (object): The vessel configuration object, containing waypoints and other params.
        """
        # Initialize the ROS2 Node with a unique name based on vessel
        node_name = f'guidance_control_{vessel.vessel_name}_{vessel.vessel_id:02d}'
        super().__init__(node_name)
        self.get_logger().info(f"Initializing Guidance Control Node: {node_name}")

        self.vessel = vessel
        # Define topic names based on vessel name and ID for namespacing
        self.topic_prefix = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}'
        self.odom_topic = f'/{self.topic_prefix}/odometry'
        self.actuator_topic = f'/{self.topic_prefix}/actuator_cmd'
        self.waypoints_topic = f'/{self.topic_prefix}/waypoints' # Topic for publishing waypoints
        
        # --- Subscribers and Publishers ---
        # Create subscriber for vessel odometry data
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback, # Callback function to process odometry
            10) # QoS history depth
        self.get_logger().info(f"Subscribing to Odometry on: {self.odom_topic}")
            
        # Create publisher for actuator (rudder) commands
        self.actuator_pub = self.create_publisher(
            Actuator, # Custom message type
            self.actuator_topic,
            10) # QoS history depth
        self.get_logger().info(f"Publishing Actuator commands on: {self.actuator_topic}")
        
        # Create publisher for visualizing the waypoints (e.g., in RViz)
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            self.waypoints_topic,
            10) # QoS history depth, potentially latched if waypoints don't change?
        self.get_logger().info(f"Publishing Waypoints (PoseArray) on: {self.waypoints_topic}")
        
        # Create a timer to periodically publish the waypoints
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every 1 second
        
        # --- Waypoint and Control Setup ---
        # Read waypoints from the vessel configuration dictionary
        try:
            self.waypoints = np.array(self.vessel.vessel_config['guidance']['waypoints'])
            self.waypoints_type = self.vessel.vessel_config['guidance'].get('waypoints_type', 'absolute') # Default type
            if len(self.waypoints) < 2:
                self.get_logger().error("Insufficient waypoints defined (need at least 2 for path following).")
                # Handle error appropriately, e.g., shutdown or default behavior
                raise ValueError("Need at least 2 waypoints.")
            self.waypoint_idx = 1 # Start targeting the second waypoint (index 1)
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints (type: {self.waypoints_type}). Initial target index: {self.waypoint_idx}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in vessel config guidance section: {e}. Cannot load waypoints.")
            # Handle error: shutdown, use defaults, etc.
            raise
        except Exception as e:
             self.get_logger().error(f"Error loading waypoints: {e}")
             raise

        # Store waypoints potentially for republishing (though config shouldn't change)
        # Consider making this immutable if waypoints are static
        self.waypoints_data = {
            'waypoints': self.waypoints,
            'waypoints_type': self.waypoints_type
        }
        
        # --- Initialization ---
        # Initialize time tracking for integration
        self.start_time = self.get_clock().now()

        # Initialize integrator states for control law
        self.ye_int = 0.0 # Integral of cross-track error
        self.te_int = 0.0 # Time of last integration update
        
        # Select the control mode/function from the imported module
        # Currently hardcoded to pid_control. Could be configurable.
        self.control_mode = con.pid_control 
        self.get_logger().info(f"Using control mode: {self.control_mode.__name__}")
        
        self.get_logger().info(f"Guidance Control Node {node_name} initialized successfully.")

    def publish_waypoints(self):
        """Publishes the waypoints as a PoseArray for visualization."""
        if not hasattr(self, 'waypoints_data') or not self.waypoints_data:
            self.get_logger().warning("Waypoint data not available for publishing.")
            return
            
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg() # Use current time
        # Frame ID should match the frame used by the Odometry (e.g., "map" or "odom" or "NED")
        pose_array.header.frame_id = "map" # TODO: Make this configurable or match Odometry frame_id
        
        waypoints_list = self.waypoints_data.get('waypoints', [])
        if not isinstance(waypoints_list, (list, np.ndarray)):
             self.get_logger().error("Waypoints data is not a list or numpy array.")
             return

        for i, waypoint in enumerate(waypoints_list):
            if len(waypoint) < 3:
                self.get_logger().warning(f"Waypoint {i} has less than 3 dimensions: {waypoint}. Skipping z.")
                z_coord = 0.0
            else:
                 z_coord = float(waypoint[2])
            try:
                pose = Pose()
                pose.position.x = float(waypoint[0])
                pose.position.y = float(waypoint[1])
                pose.position.z = z_coord 
                # Orientation is not set, defaults to (0,0,0,1)
                pose_array.poses.append(pose)
            except (ValueError, IndexError) as e:
                 self.get_logger().error(f"Error processing waypoint {i}: {waypoint}. Error: {e}")
        
        if len(pose_array.poses) > 0:
             self.waypoints_pub.publish(pose_array)
        # else: self.get_logger().debug("No valid waypoints to publish.")
    
    def odom_callback(self, msg: Odometry):
        """Callback function executed when new Odometry message is received."""
        # Calculate elapsed time since node start for potential use in controllers
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9 # Time in seconds

        # Extract orientation quaternion
        q_msg = msg.pose.pose.orientation
        quat = np.array([q_msg.w, q_msg.x, q_msg.y, q_msg.z])

        # Convert quaternion to Euler angles (ZYX convention often used for yaw, pitch, roll)
        try:
            # Assuming ZYX order gives [yaw, pitch, roll]. Check convention needed by control module.
            eul = quat_to_eul(quat) # Default order might be XYZ? Specify order='ZYX' if needed.
            psi = eul[2] # Assuming ZYX -> Yaw is the last element
        except Exception as e:
            self.get_logger().error(f"Quaternion to Euler conversion failed: {e}")
            return
        
        # Construct the state vector required by the control module
        # **CRITICAL**: Ensure this state vector matches the exact format expected by self.control_mode
        # Based on pid_control in module_control, it seems to expect:
        # [u, v, w, p, q, r, x, y, z, phi, theta, psi, delta? (rudder angle)]
        # Indices used: 3(u), 4(v), 5(r), 6(x), 7(y), 11(psi)
        state = np.array([
            msg.twist.twist.linear.x,    # 0: u (surge velocity)
            msg.twist.twist.linear.y,    # 1: v (sway velocity)
            msg.twist.twist.linear.z,    # 2: w (heave velocity)
            msg.twist.twist.angular.x,   # 3: p (roll rate)
            msg.twist.twist.angular.y,   # 4: q (pitch rate)
            msg.twist.twist.angular.z,   # 5: r (yaw rate)
            msg.pose.pose.position.x,    # 6: x (North position?)
            msg.pose.pose.position.y,    # 7: y (East position?)
            msg.pose.pose.position.z,    # 8: z (Down position?)
            eul[0],                      # 9: phi/roll ? (Check quat_to_eul order)
            eul[1],                      # 10: theta/pitch ?
            eul[2],                      # 11: psi/yaw ?
            0.0                          # 12: Placeholder for rudder angle (not used as input to pid_control)
        ])

        # Check if all waypoints have been processed
        if self.waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached. No further control commands issued.")
            # Optionally publish zero command or stop publishing
            cmd_msg = Actuator()
            cmd_msg.actuator_values = [0.0]
            cmd_msg.actuator_names = ['cs_1'] # Assuming 'cs_1' is the rudder
            self.actuator_pub.publish(cmd_msg)
            return

        # Calculate control command using the selected control function
        try:
            # Pass current time, state, waypoints array, target index, and integrated error
            rudder_cmd_rad, ye, wp_indx = self.control_mode(t, state, self.waypoints, self.waypoint_idx, self.ye_int)
        except Exception as e:
             self.get_logger().error(f"Error during control calculation ({self.control_mode.__name__}): {e}")
             return

        # Check if the control function advanced the waypoint index
        if wp_indx != self.waypoint_idx:
            self.get_logger().info(f'Waypoint {self.waypoint_idx} reached. Switched target to index {wp_indx}')
            self.waypoint_idx = wp_indx
            self.ye_int = 0.0 # Reset integrator on waypoint switch
            # Check if this was the last waypoint
            if self.waypoint_idx >= len(self.waypoints):
                 self.get_logger().info("Last waypoint reached.")
                 # Allow one more cycle to publish final command if needed, then stop

        # Update the integral of the cross-track error (ye_int)
        dt = t - self.te_int # Time difference since last update
        if dt > 1e-6:  # Avoid division by zero or negative time, ensure some time passed
            self.ye_int += ye * dt # Simple Euler integration
            self.te_int = t # Update timestamp of last integration
        
        # --- Publish Command --- 
        cmd_msg = Actuator()
        # Assume control law returns radians, convert to degrees for actuator message?
        # **CHECK** actuator interface: does it expect radians or degrees?
        # Assuming degrees based on the multiplication by 180/pi.
        rudder_cmd_deg = float(rudder_cmd_rad * 180.0 / np.pi)
        cmd_msg.actuator_values = [rudder_cmd_deg] 
        # Name of the actuator being commanded (e.g., control surface 1)
        # TODO: Make actuator name configurable or read from vessel params
        cmd_msg.actuator_names = ['cs_1'] 
        # Covariance - currently zero, indicating high confidence? Or unused.
        cmd_msg.covariance = [0.0] 
        self.actuator_pub.publish(cmd_msg)
        
        # Optional logging
        # self.get_logger().info(
        #     f't={t:.2f} | WP Idx: {self.waypoint_idx} | ye={ye:.2f} | ye_int={self.ye_int:.2f} | Rudder Cmd={rudder_cmd_deg:.2f} deg'
        # )