#!/usr/bin/env python3

import numpy as np
import sys
sys.path.append('/workspaces/makara/ros2_ws/install/mav_simulator/lib/python3.10/site-packages/mav_simulator/')

from class_world import World
import module_shared as sh
import asyncio
import json
import websockets
import rclpy
from class_world_node_ros2 import World_Node
import module_kinematics as kin



def main():
    # Creates an object of class 'World'
    sh.world = World('/workspaces/makara/ros2_ws/src/mav_simulator/mav_simulator/world_file.yml')

    rclpy.init()
    world_node = World_Node(world_rate=1/sh.dt)
    sh.world.node = world_node
    sh.world.start_vessel_ros_nodes()
    rclpy.spin(world_node)
    rclpy.shutdown()


    # rclpy.init()
    # sh.world.node = World_Node()
    # sh.world.start_vessel_ros_nodes()

    # async def handler(websocket):

    #     while rclpy.ok():
    #         data = []
    #         for vessel in sh.world.vessels:
    #             state=vessel.current_state
    #             eul=kin.quat_to_eul(state[9:13])
    #             state_eul=state[6:9].tolist()+eul.tolist()
    #             data.append(state_eul)

    #         await websocket.send(json.dumps(data))
    #         # await asyncio.sleep(0.01)
    #         # sh.world.step()
    #         rclpy.spin_once(sh.world.node)
    #     rclpy.shutdown()

    #     # Send subsequent lists, one element at a tim

    # start_server = websockets.serve(handler, "localhost", 9000)

    # asyncio.get_event_loop().run_until_complete(start_server)
    # asyncio.get_event_loop().run_forever()

if __name__ == '__main__':
    print('$(pwd)')    
    main()



