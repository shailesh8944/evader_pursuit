import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
# from interfaces.msg import Actuator
import module_kinematics as kin

class BaseSensor:
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        self.vessel_id = vessel_id
        self.topic_prefix = topic_prefix
        self.sensor_type = sensor_config['sensor_type']
        self.rate = sensor_config['publish_rate']
        self.topic = None
        self.id = sensor_config.get('id', 0)
        self.vessel_node = vessel_node

class IMUSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        self.orientation = np.array(sensor_config['sensor_orientation'])
        
        # Noise parameters
        self.eul_rms = np.array([1, 1, 1]) * 1e-2
        self.eul_cov = np.diag(self.eul_rms ** 2)
        self.ang_vel_rms = np.array([1, 1, 1]) * 1e-2
        self.ang_vel_cov = np.diag(self.ang_vel_rms ** 2)
        self.lin_acc_rms = np.array([1, 1, 1]) * 1.5e-1
        self.lin_acc_cov = np.diag(self.lin_acc_rms ** 2)

    def get_measurement(self,quat=False):
        state = self.vessel_node.vessel.current_state
        state_der = self.vessel_node.vessel.current_state_der
        
        # Handle both quaternion and euler angle inputs
        if quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            quat = kin.eul_to_quat(eul)
            orientation_eul = self.orientation
            orientation_quat = kin.eul_to_quat(orientation_eul)
        else:
            quat = state[9:13]
            orientation_quat = self.orientation

        omg_bcs = state[3:6]
        v_bcs = state[0:3]        
        alpha = state_der[3:6]

        q_sensor = kin.rotm_to_quat(kin.quat_to_rotm(quat) @ kin.quat_to_rotm(orientation_quat))
        q_sensor = kin.eul_to_quat(kin.quat_to_eul(q_sensor) + np.random.multivariate_normal(np.zeros(3), self.eul_cov))
        
        acc_bcs = state_der[0:3] + np.cross(omg_bcs, v_bcs)
        acc_s_bcs = acc_bcs + np.cross(alpha, self.location) + np.cross(omg_bcs, np.cross(omg_bcs, self.location))
        acc_sensor = kin.quat_to_rotm(orientation_quat).T @ acc_s_bcs
        acc_sensor = acc_sensor + kin.quat_to_rotm(q_sensor).T @ np.array([0, 0, -self.vessel_node.vessel.g])
        acc_sensor = acc_sensor + np.random.multivariate_normal(np.zeros(3), self.lin_acc_cov)

        omg_sensor = kin.quat_to_rotm(orientation_quat).T @ omg_bcs
        omg_sensor = omg_sensor + np.random.multivariate_normal(np.zeros(3), self.ang_vel_cov)

        return {
            'orientation': q_sensor,
            'angular_velocity': omg_sensor,
            'linear_acceleration': acc_sensor,
            'orientation_covariance': self.eul_cov.flatten(),
            'angular_velocity_covariance': self.ang_vel_cov.flatten(),
            'linear_acceleration_covariance': self.lin_acc_cov.flatten()
        }

class GPSSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Noise parameters
        self.gps_rms = np.array([3, 3, 3], dtype=np.float64)
        self.gps_cov = np.diag(self.gps_rms ** 2)

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        llh0 = self.vessel_node.vessel.gps_datum
        
        # Handle both quaternion and euler angle inputs
        if not quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            orientation = kin.eul_to_quat(eul)
        else:
            orientation = state[9:13]
            
        ned = state[6:9] + kin.quat_to_rotm(orientation) @ self.location
        ned = ned + np.random.multivariate_normal(np.zeros(3), self.gps_cov)
        llh = kin.ned_to_llh(ned, llh0)

        return {
            'latitude': llh[0],
            'longitude': llh[1],
            'altitude': llh[2],
            'position_covariance': self.gps_cov.flatten()
        }

class UWBSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Noise parameters
        self.uwb_rms = np.array([1, 1, 1], dtype=np.float64)
        self.uwb_cov = np.diag(self.uwb_rms ** 2)
        self.uwb_cov_full = -np.eye(6)
        self.uwb_cov_full[0:3][:, 0:3] = self.uwb_cov

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        ned = state[6:9]
        
        # Handle both quaternion and euler angle inputs
        if not quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            orientation = kin.eul_to_quat(eul)
        else:
            orientation = state[9:13]
            
        r_sen = ned + kin.quat_to_rotm(orientation) @ self.location
        r_sen = r_sen + np.random.multivariate_normal(np.zeros(3), self.uwb_cov)

        return {
            'position': r_sen,
            'covariance': self.uwb_cov_full.flatten()
        }

class EncoderSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        
        # Get number of control surfaces and thrusters from vessel
        self.num_control_surfaces = len(vessel_node.vessel.control_surfaces['control_surfaces'])
        self.num_thrusters = len(vessel_node.vessel.thrusters['thrusters'])
        
        # Calculate indices in state vector for actuators
        # From vessel_ode: state vector structure is:
        # - state[0:6]: Velocities [u, v, w, p, q, r]
        # - state[6:9]: Positions [x, y, z]
        # - state[9:12/13]: Euler angles [phi, theta, psi] or Quaternions [q0, q1, q2, q3]
        # - state[control_start:thruster_start]: Control surface angles
        # - state[thruster_start:]: Thruster states
        use_quaternion = vessel_node.vessel.use_quaternion
        attitude_size = 4 if use_quaternion else 3
        self.control_start = 9 + attitude_size
        self.thruster_start = self.control_start + self.num_control_surfaces
        
        # Create noise parameters for all actuators
        # Control surfaces in radians, thrusters in RPM
        self.encoders_rms = np.concatenate([
            np.ones(self.num_control_surfaces) * 1e-1 * np.pi / 180,  # 0.1 deg for control surfaces
            np.ones(self.num_thrusters) * 1e-1 / 60  # 0.1 RPM for thrusters
        ])
        self.encoders_cov = np.diag(self.encoders_rms ** 2)

    def get_measurement(self):
        state = self.vessel_node.vessel.current_state
        
        # Get control surface angles from state vector (convert to degrees)
        control_surface_values = [
            state[self.control_start + i] * 180.0 / np.pi 
            for i in range(self.num_control_surfaces)
        ]
        
        # Get thruster RPMs from state vector (convert to RPM)
        thruster_values = [
            state[self.thruster_start + i] * 60.0 
            for i in range(self.num_thrusters)
        ]
        
        # Add noise to measurements
        actuator_values = np.concatenate([control_surface_values, thruster_values])
        actuator_values += np.random.multivariate_normal(np.zeros(len(actuator_values)), self.encoders_cov)
        
        # Split back into control surfaces and thrusters
        control_surface_values = actuator_values[:self.num_control_surfaces].tolist()
        thruster_values = actuator_values[self.num_control_surfaces:].tolist()
        
        # Create actuator names
        control_surface_names = [f'cs_{i+1}' for i in range(self.num_control_surfaces)]
        thruster_names = [f'th_{i+1}' for i in range(self.num_thrusters)]
        
        return {
            'actuator_values': control_surface_values + thruster_values,
            'actuator_names': control_surface_names + thruster_names,
            'covariance': self.encoders_cov.flatten()
        }

class DVLSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        
        # Noise parameters - typical DVL has accuracy of ~0.2-1% of measured velocity
        self.vel_rms = np.array([0.05, 0.05, 0.05])  # 5cm/s RMS noise in each axis
        self.vel_cov = np.diag(self.vel_rms ** 2)

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        
        # Get body-frame velocity and add noise
        v_body = state[0:3]
        v_body_noisy = v_body + np.random.multivariate_normal(np.zeros(3), self.vel_cov)
        
        return {
            'velocity': v_body_noisy,
            'covariance': self.vel_cov.flatten()  # Return 3x3 covariance matrix
        }

def create_sensor(sensor_config, vessel_id, topic_prefix, vessel_node):
    """Factory function to create appropriate sensor object based on sensor type"""
    sensor_type = sensor_config['sensor_type']
    sensor_classes = {
        'IMU': IMUSensor,
        'GPS': GPSSensor,
        'UWB': UWBSensor,
        'encoders': EncoderSensor,
        'DVL': DVLSensor
    }
    
    if sensor_type not in sensor_classes:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
        
    return sensor_classes[sensor_type](sensor_config, vessel_id, topic_prefix, vessel_node) 