from rclpy.node import Node
from class_world import World

class World_Node(Node):
    rate = None
    def __init__(self, world_rate=100, world_file=None):
        super().__init__('world')
        self.rate = world_rate
        self.world = World(world_file)
        self.create_timer(1/self.rate, callback=self.world.step)