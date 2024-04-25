#!/usr/bin/env python3

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
import warnings
from class_world_node_ros2 import World_Node
import websockets
import asyncio
import json

class GNC():
    sensor_sub = []
    odom_pub = None
    actuator = {}
    rate = 10
    node = None
    topic_prefix = None
    actuator_cmd = None

    def __init__(self, topic_prefix, rate=10):
        self.topic_prefix = topic_prefix
        self.rate = rate

    def register_actuator(self):
        self.actuator['pub'] = self.node.create_publisher(Actuator, f'{self.topic_prefix}/actuator_cmd', self.rate)
        self.actuator['timer'] = self.node.create_timer(1/self.rate, self.publish_actuator)


    def register_sensor(self, sensors):
        for sensor in sensors:
            if sensor['sensor_type'] == "IMU":
                h = self.node.create_subscription(Imu, sensor['topic'], self.imu_callback, 1)
            elif sensor['sensor_type'] == "GPS":
                h = self.node.create_subscription(NavSatFix, sensor['topic'], self.gps_callback, 1)
            elif sensor['sensor_type'] == "UWB":
                h = self.node.create_subscription(PointStamped, sensor['topic'], self.uwb_callback, 1)
            else:
                h = None
                warnings.warn(f"Sensor type {sensor['sensor_type']} is unknown")
            sensor_sub.append(h)
    
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
    
    def publish_actuator(self):
        current_time = sh.world.node.get_clock().now()

        # Create odometry message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        act.rudder = 35 * np.pi / 180
        act.propeller = 13.33

        self.actuator['pub'].publish(act)

        self.actuator_cmd = f'{act.rudder * 180 / np.pi:.2f}, {np.random.random() * 60}'
        self.node.get_logger().info(self.actuator_cmd)

        
def main():
    # rclpy.init()
    # sh.world = GNC(topic_prefix='makara_00', rate=10)
    # sh.world.node = Node('Test')
    # sh.world.register_actuator()
    # rclpy.spin(sh.world.node)
    # rclpy.shutdown()

    rclpy.init()
    sh.world = GNC(topic_prefix='makara_00', rate=10)
    sh.world.node = Node('Test')

    async def handler(websocket):

        while rclpy.ok():
            print(sh.world.actuator_cmd)
            await websocket.send(sh.world.actuator_cmd)
            # await websocket.send('Hello Vallabh')
            rclpy.spin_once(sh.world.node)
        rclpy.shutdown()

        # Send subsequent lists, one element at a tim

    start_server = websockets.serve(handler, "0.0.0.0", 9000)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


    # No ROS is working fine
    # async def handler(websocket):
    #     while True:
    #         await websocket.send(f'Hello Vallabh {np.random.random()*10:.2f}')
    
    # start_server = websockets.serve(handler, "0.0.0.0", 9000)

    # asyncio.get_event_loop().run_until_complete(start_server)
    # asyncio.get_event_loop().run_forever()
    
if __name__ == "__main__":
    main()