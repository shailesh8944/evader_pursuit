import rclpy
from rclpy.node import Node
import module_shared as sh

class World_Node(Node):
    def __init__(self, world_rate=100):
        super().__init__('world')
        self.create_timer(1/world_rate, callback=sh.world.step)