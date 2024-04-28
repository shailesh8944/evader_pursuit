#!/usr/bin/env python3

import numpy as np
import rclpy
import websockets
import asyncio
import json
import threading
from class_gnc import GNC

def ros_thread(gnc):
    rclpy.spin(gnc.node)
    rclpy.shutdown()

def main():
    # rclpy.init()
    # sh.world = GNC(topic_prefix='makara_00', rate=10)
    # sh.world.node = Node('Test')
    # sh.world.register_actuator()
    # rclpy.spin(sh.world.node)
    # rclpy.shutdown()

    rclpy.init()
    gnc = GNC(topic_prefix='makara_00', rate=10)

    sensors = []
    sensor1 = {}
    sensor1['sensor_type'] = 'IMU'
    sensor1['topic'] = '/makara_00/imu_00'

    sensor2 = {}
    sensor2['sensor_type'] = 'UWB'
    sensor2['topic'] = 'makara_00/uwb_01'
    
    sensor3 = {}
    sensor3['sensor_type'] = 'encoders'
    sensor3['topic'] = '/makara_00/actuator_02'

    sensors = [sensor1, sensor2, sensor3]
    gnc.register_sensors(sensors)

    ros_thread_instance = threading.Thread(target=ros_thread, args=(gnc,))
    ros_thread_instance.start()

    async def handler(websocket):
        while True:
            await websocket.send(gnc.actuator_cmd)
            await asyncio.sleep(1/(gnc.rate))

    start_server = websockets.serve(handler, "0.0.0.0", 9001)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
    
if __name__ == "__main__":
    main()