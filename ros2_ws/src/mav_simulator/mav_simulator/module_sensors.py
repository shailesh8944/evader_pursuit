import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
from interfaces.msg import Actuator
import module_shared as sh
import module_kinematics as kin

class BaseSensor:
    def __init__(self, sensor_config, vessel_id, topic_prefix):
        self.vessel_id = vessel_id
        self.topic_prefix = topic_prefix
        self.sensor_type = sensor_config['sensor_type']
        self.rate = sensor_config['publish_rate']
        self.topic = sensor_config['topic']
        self.id = sensor_config.get('id', 0)

class IMUSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix):
        super().__init__(sensor_config, vessel_id, topic_prefix)
        self.location = np.array(sensor_config['sensor_location'])
        self.orientation = np.array(sensor_config['sensor_orientation'])
        
        # Noise parameters
        self.eul_rms = np.array([1, 1, 1]) * 1e-2
        self.eul_cov = np.diag(self.eul_rms ** 2)
        self.ang_vel_rms = np.array([1, 1, 1]) * 1e-2
        self.ang_vel_cov = np.diag(self.ang_vel_rms ** 2)
        self.lin_acc_rms = np.array([1, 1, 1]) * 1.5e-1
        self.lin_acc_cov = np.diag(self.lin_acc_rms ** 2)

    def get_measurement(self):
        state = sh.world.vessels[self.vessel_id].current_state
        state_der = sh.world.vessels[self.vessel_id].current_state_der
        
        # Handle both quaternion and euler angle inputs
        if hasattr(self, 'quat') and not self.quat:
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
        acc_sensor = acc_sensor + kin.quat_to_rotm(q_sensor).T @ np.array([0, 0, -sh.g])
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
    def __init__(self, sensor_config, vessel_id, topic_prefix):
        super().__init__(sensor_config, vessel_id, topic_prefix)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Noise parameters
        self.gps_rms = np.array([3, 3, 3], dtype=np.float64)
        self.gps_cov = np.diag(self.gps_rms ** 2)

    def get_measurement(self):
        state = sh.world.vessels[self.vessel_id].current_state
        llh0 = sh.world.gps_datum
        ned = state[6:9] + kin.quat_to_rotm(state[9:13]) @ self.location
        ned = ned + np.random.multivariate_normal(np.zeros(3), self.gps_cov)
        llh = kin.ned_to_llh(ned, llh0)

        return {
            'latitude': llh[0],
            'longitude': llh[1],
            'altitude': llh[2],
            'position_covariance': self.gps_cov.flatten()
        }

class UWBSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix):
        super().__init__(sensor_config, vessel_id, topic_prefix)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Noise parameters
        self.uwb_rms = np.array([1, 1, 1], dtype=np.float64)
        self.uwb_cov = np.diag(self.uwb_rms ** 2)
        self.uwb_cov_full = -np.eye(6)
        self.uwb_cov_full[0:3][:, 0:3] = self.uwb_cov

    def get_measurement(self):
        state = sh.world.vessels[self.vessel_id].current_state
        ned = state[6:9]
        r_sen = ned + kin.quat_to_rotm(state[9:13]) @ self.location
        r_sen = r_sen + np.random.multivariate_normal(np.zeros(3), self.uwb_cov)

        return {
            'position': r_sen,
            'covariance': self.uwb_cov_full.flatten()
        }

class EncoderSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix):
        super().__init__(sensor_config, vessel_id, topic_prefix)
        
        # Noise parameters
        self.encoders_rms = np.array([1e-1 * np.pi / 180, 1e-1 / 60])
        self.encoders_cov = np.diag(self.encoders_rms ** 2)

    def get_measurement(self):
        state = sh.world.vessels[self.vessel_id].current_state
        
        return {
            'rudder': state[13] * 180.0 / np.pi,
            'propeller': state[14] * 60.0,
            'covariance': self.encoders_cov.flatten()
        }

def create_sensor(sensor_config, vessel_id, topic_prefix):
    """Factory function to create appropriate sensor object based on sensor type"""
    sensor_type = sensor_config['sensor_type']
    sensor_classes = {
        'IMU': IMUSensor,
        'GPS': GPSSensor,
        'UWB': UWBSensor,
        'encoders': EncoderSensor
    }
    
    if sensor_type not in sensor_classes:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
        
    return sensor_classes[sensor_type](sensor_config, vessel_id, topic_prefix) 