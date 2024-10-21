#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
import asyncio
import websockets
import json
import logging

class WebSocketServer(Node):
    def __init__(self):
        super().__init__('websocket_server')
        self.odom_subscriber = self.create_subscription(
            Odometry, '/makara_00/odometry', self.odom_callback, 10)
        # self.buoy_subscriber = self.create_subscription(
        #     Float32MultiArray, '/waypoints/buoy_array', self.buoy_callback, 10)
        # self.create_subscription(PoseStamped, '/waypoints/red_buoy', self.red_buoy_callback, 10)
        # self.create_subscription(PoseStamped, '/waypoints/green_buoy', self.green_buoy_callback, 10)
        # self.create_subscription(Float32MultiArray,'/battery/voltage', self.battery_voltage_callback,10)
        # self.create_subscription(PoseStamped, '/waypoints/cardinal_buoy', self.cardinal_buoy_callback, 10)
        # self.create_subscription(PoseStamped, '/waypoints/aruco_pose', self.arucopose_callback, 10)
        # self.create_subscription(PoseArray, '/gui/green_buoy_used', self.green_buoy_used_callback, 10)
        # self.create_subscription(PoseArray, '/gui/red_buoy_used', self.red_buoy_used_callback, 10)
        
        self.odom_data = {
            'x': 0,
            'y': 0,
            'yaw': 0,
            'pitch':0,
            'roll':0


        }
        # Simulate buoy data
        # self.red_buoy = {
        #     'x': 0,
        #     'y': 0,
        #     'type': 0,
        # }
        # self.green_buoy = {
        #     'x': 0,
        #     'y': 0,
        #     'type': 0,
        # }
        
        # self.battery_voltage ={
        #     'Battery_1': 0.0,
        #     'Battery_2': 0.0,
        #     'Battery_3': 0.0

        # }
        # self.aruco_pose = {
        #     'x': 0,
        #     'y': 0,
        #     'type': 0,
        # }
        # self.cardinal_buoy = {
        #     'x': 0,
        #     'y': 0,
        #     'type': 0,
        # }
    def odom_callback(self, msg):
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        yaw = R.from_quat(orientation).as_euler('xyz')[2]
        pitch = R.from_quat(orientation).as_euler('xyz')[1]
        roll = R.from_quat(orientation).as_euler('xyz')[1]

        self.odom_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'pitch': pitch,
            'roll' : roll,
            'v_x': msg.twist.twist.linear.x,
            'v_y': msg.twist.twist.linear.y
        }
       
        self.get_logger().info(f"Received odometry data: {self.odom_data}")

    # def red_buoy_callback(self, msg):
    #     self.red_buoy = {
    #         'x': msg.pose.position.x,
    #         'y': msg.pose.position.y,
    #         'type': 2
    #     }
    # def green_buoy_callback(self, msg):
    #     self.green_buoy = {
    #         'x': msg.pose.position.x,
    #         'y': msg.pose.position.y,
    #         'type': 1
    #     }

    # def battery_voltage_callback(self,msg):
    #     self.battery_voltage ={
    #         'Battery_1': msg.data[0],
    #         'Battery_2': msg.data[1],
    #         'Battery_3': msg.data[2]

    #     }
    # def cardinal_buoy_callback(self, msg):
    #     self.cardinal_buoy = {
    #         'x': msg.pose.position.x,
    #         'y': msg.pose.position.y,
    #         'type': msg.pose.position.z
    #     }
    # def arucopose_callback(self, msg):
    #     self.arucopose = {
    #         'x': msg.pose.position.x,
    #         'y': msg.pose.position.y,
    #         'type': 7
    #     }
    # def green_buoy_used_callback(self, msg):
    #     self.green_used_buoy = []
    #     for indv_pose in msg.poses:
    #         self.green_used_buoy.append([indv_pose.pose.position.x, indv_pose.pose.position.y])
    
    # def red_buoy_used_callback(self, msg):
    #     self.red_used_buoy = []
    #     for indv_pose in msg.poses:
    #         self.red_used_buoy.append([indv_pose.pose.position.x, indv_pose.pose.position.y])

def main(args=None):
    rclpy.init(args=args)
    websocket_server = WebSocketServer()

    async def handler(websocket, path):

        try:
            while rclpy.ok():
                message = json.dumps({'odometry': websocket_server.odom_data})
                #                           'battery': websocket_server.battery_voltage,\
                #                         'cardinal_buoy': websocket_server.cardinal_buoy, 'arucopose': websocket_server.aruco_pose, \
                                        # 'green_used_buoy': websocket_server.green_used_buoy, 'red_used_buoy': websocket_server.red_used_buoy, \
                                        
                # message = json.dumps({'red_buoy': websocket_server.red_buoy, 'green_buoy': websocket_server.green_buoy, \
                #                         'cardinal_buoy': websocket_server.cardinal_buoy, 'arucopose': websocket_server.arucopose, \
                #                         'green_used_buoy': websocket_server.green_used_buoy, 'red_used_buoy': websocket_server.red_used_buoy, \
                #                         'odometry': websocket_server.odom_data})
                await websocket.send(message)
                websocket_server.get_logger().info(f"Sent: {message}")
                # await asyncio.sleep(0.01)
                rclpy.spin_once(websocket_server, timeout_sec=0.1)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            pass

    start_server = websockets.serve(handler, "0.0.0.0", 9000)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()




if __name__ == '__main__':
    main()

