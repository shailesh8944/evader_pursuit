#!/usr/bin/env python3

import rospy
import websocket
from std_msgs.msg import String
import time

class WebSocketROSBridge_Actuator:
    def __init__(self, node, websocket_url, ros_topic):
        # Initialize ROS node
        self.node = node
        self.websocket_url = websocket_url
        self.ros_topic = ros_topic

        # Create a publisher for the ROS topic
        self.pub = rospy.Publisher(self.ros_topic, String, queue_size=10)

    def on_message(self, ws, message):
        # Publish the received message to the ROS topic
        # rospy.loginfo("Received message from WebSocket: %s", message)
        self.pub.publish(message)

    def on_error(self, ws, error):
        rospy.logerr(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        rospy.loginfo(f"WebSocket closed with status code {close_status_code}: {close_msg}")

    def on_open(self, ws):
        rospy.loginfo("WebSocket connected")

    def run(self):
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
    bridge = WebSocketROSBridge_Actuator(websocket_url='ws://localhost:9001',
                                ros_topic='/ASV')
    bridge.run()
    
