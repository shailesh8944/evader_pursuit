#!/usr/bin/env python3

import numpy as np
import rospy
import websocket
import json
import asyncio
from std_msgs.msg import String
import time

class WebSocketROSBridge_Encoder:
    def __init__(self, node, websocket_url, topic_prefix='', HIL_flag=False):
        # Initialize ROS node
        self.node = node
        self.websocket_url = websocket_url
        self.topic_prefix = topic_prefix
        self.HIL_flag = HIL_flag

        self.rudder = 0.0
        self.propeller = 0.0

        # Create a publisher for the ROS topic
        if self.HIL_flag:
            self.sub = rospy.Subscriber('/absoluteEncoder', String, self.encoder_ros1_callback)
            
    def encoder_ros1_callback(self, msg):
        data = msg.data.split(',')
        self.rudder = np.float64(data[0])
        self.propeller = np.float64(data[1])

    def on_message(self, ws, message):
        msg = json.loads(message)
        if msg['vessel'] == self.topic_prefix:
            self.rudder = np.float64(msg['rudder'])
            self.propeller = np.float64(msg['propeller'])
            # print(f'Simulation tells -> Rudder: {self.rudder:.2f}, Propeller: {self.propeller:.2f}')

    def on_error(self, ws, error):
        rospy.logerr(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        rospy.loginfo(f"WebSocket closed with status code {close_status_code}: {close_msg}")

    def on_open(self, ws):
        rospy.loginfo("WebSocket connected")

    def run(self):
        if self.HIL_flag:
            rospy.spin()
        else:
            while True:
                time.sleep(5)
                try:
                    # Connect to the WebSocket server
                    # websocket.enableTrace(True)  # Enable trace for debugging (optional)
                    ws = websocket.WebSocketApp(self.websocket_url,
                                            on_message=self.on_message,
                                            on_error=self.on_error,
                                            on_close=self.on_close)
                    ws.on_open = self.on_open
                    ws.run_forever()
                except:
                    pass

if __name__ == '__main__':

    # Replace 'ws://example.com/socket' with your WebSocket URL
    # Replace '/ros_topic' with the name of your ROS topic
    bridge = WebSocketROSBridge_Encoder(websocket_url='ws://localhost:9002',
                                topic_prefix='makara_00', HIL_flag=True)
    bridge.run()

    
    
