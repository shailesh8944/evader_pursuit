#!/usr/bin/env python3

import numpy as np
import rospy
import websocket
import websockets
import json
import asyncio
import threading
from actuator_publisher import WebSocketROSBridge_Actuator
from encoder_listener import WebSocketROSBridge_Encoder


class RasPi():
    bridge1 = None
    bridge2 = None
    thread1 = None
    thread2 = None

    def __init__(self):
        pass

def main(raspi):

    raspi.bridge1 = WebSocketROSBridge_Actuator(websocket_url='ws://192.168.0.6:9001',
                                ros_topic='/ASV')
    raspi.thread1 = threading.Thread(target=raspi.bridge1.run)
    raspi.thread1.start()

    raspi.bridge2 = WebSocketROSBridge_Encoder(websocket_url='ws://localhost:9002',
                                topic_prefix='makara_00', simulator_flag=False)
    raspi.thread2 = threading.Thread(target=raspi.bridge2.run)
    raspi.thread2.start()

    # Run asyncio on a separate thread
    async def handler(websocket):
        while True:
            data = {}
            data['rudder'] = raspi.bridge2.rudder
            data['propeller'] = raspi.bridge2.propeller

            await websocket.send(json.dumps(data))
            await asyncio.sleep(1.0/30.0)

    start_server = websockets.serve(handler, "0.0.0.0", 9003)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    try:
        rospy.init_node('raspi', anonymous=True)
        raspi = RasPi()
        main(raspi)
    finally:
        raspi.bridge1.on_message(None, "0.0, 0.0")



