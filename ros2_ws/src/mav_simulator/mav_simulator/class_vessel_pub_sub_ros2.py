import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PointStamped
from interfaces.msg import Actuator
import module_shared as sh
import module_kinematics as kin

class Vessel_Pub_Sub():
    sensors = []
    actuators = []
    vessel_id = None
    vessel_name = None
    topic_prefix = None
    n_c = 0         # Test turning circle -> 40/3
    delta_c = 0     # Test turning circle -> 35 * np.pi / 180

    def __init__(self, vessel_id):
        self.vessel_id = vessel_id
        self.vessel_name = sh.world.vessels[self.vessel_id].name
        self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'
        for sensor in sh.world.vessels[self.vessel_id].sensors:
            self.register_sensor(sensor)
        self.register_actuators()
    
    def register_actuators(self):
        self.actuators.append(sh.world.node.create_subscription(Actuator, f'{self.topic_prefix}/actuator_cmd', self.actuator_callback, 1))

    def register_sensor(self, sensor):
        rate = sensor['publish_rate']
        loc = np.array(sensor['sensor_location'])
        orient = np.array(sensor['sensor_orientation'])

        sensor['id'] = len(self.sensors)
        self.sensors.append(sensor)

        if sensor['sensor_type'] == 'IMU':
            self.sensors[-1]['pub'] = sh.world.node.create_publisher(Imu, f'{self.topic_prefix}/imu_{sensor["id"]:02d}', rate)
            self.sensors[-1]['timer'] = sh.world.node.create_timer(1/rate, lambda: self.publish_imu(id=sensor['id'], loc=loc, orient=orient))

        elif sensor['sensor_type'] == 'GPS':
            self.sensors[-1]['pub'] = sh.world.node.create_publisher(NavSatFix, f'{self.topic_prefix}/gps_{sensor["id"]:02d}', rate)
            self.sensors[-1]['timer'] = sh.world.node.create_timer(1/rate, lambda: self.publish_gps(id=sensor['id'], loc=loc))
        
        elif sensor['sensor_type'] == 'UWB':
            self.sensors[-1]['pub'] = sh.world.node.create_publisher(PointStamped, f'{self.topic_prefix}/uwb_{sensor["id"]:02d}', rate)
            self.sensors[-1]['timer'] = sh.world.node.create_timer(1/rate, lambda: self.publish_uwb(id=sensor['id'], loc=loc))
        else:
            raise ValueError("Specified sensor type is unknown")
        
    def publish_imu(self, id, loc, orient):
        current_time = sh.world.node.get_clock().now()

        # Create odometry message
        imu = Imu()
        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = self.topic_prefix + f'_imu_{id:02d}_frame'

        state = sh.world.vessels[self.vessel_id].current_state
        state_der = sh.world.vessels[self.vessel_id].current_state_der
        
        quat = state[9:13]
        omg_bcs = state[3:6]
        v_bcs = state[0:3]        
        alpha = state_der[3:6]
        
        acc_bcs = state_der[0:3] + np.cross(omg_bcs, v_bcs)
        acc_s_bcs = acc_bcs + np.cross(alpha, loc) + np.cross(omg_bcs, np.cross(omg_bcs, loc))
        acc_sensor = kin.quat_to_rotm(orient).T @ acc_s_bcs
        omg_sensor = kin.quat_to_rotm(orient).T @ omg_bcs
        q_sensor = kin.rotm_to_quat(kin.quat_to_rotm(quat) @ kin.quat_to_rotm(orient))        

        imu.orientation = Quaternion(x=q_sensor[1], y=q_sensor[2], z=q_sensor[3], w=q_sensor[0])
        imu.angular_velocity = Vector3(x=omg_sensor[0], y=omg_sensor[1], z=omg_sensor[2])
        imu.linear_acceleration = Vector3(x=acc_sensor[0], y=acc_sensor[1], z=acc_sensor[2])

        self.sensors[id]['pub'].publish(imu)

    def publish_gps(self, id, loc):
        current_time = sh.world.node.get_clock().now()

        state = sh.world.vessels[self.vessel_id].current_state
        llh0 = np.array([12.993661, 80.239402, 0])
        ned = state[0:3] + kin.quat_to_rotm(state[9:13]) @ loc
        llh = kin.ned_to_llh(ned, llh0)

        # Create NavSatFix message
        gps = NavSatFix()
        gps.header.stamp = current_time.to_msg()
        gps.status = NavSatStatus(status=0, service=1)
        gps.latitude = llh[0]
        gps.longitude = llh[1]
        gps.altitude = llh[2]

        self.sensors[id]['pub'].publish(gps)
    
    def publish_uwb(self, id, loc):
        current_time = sh.world.node.get_clock().now()

        state = sh.world.vessels[self.vessel_id].current_state
        ned = state[0:3]
        r_sen = ned + kin.quat_to_rotm(state[9:13]) @ loc

        # Create UWB PointStamped message
        uwb = PointStamped()
        uwb.header.stamp = current_time.to_msg()
        uwb.point = Point(x=r_sen[0], y=r_sen[1], z=r_sen[2])
        
        self.sensors[id]['pub'].publish(uwb)

    def actuator_callback(self, msg):
        self.delta_c = msg.rudder * np.pi() / 180
        self.n_c = msg.propeller / 60
