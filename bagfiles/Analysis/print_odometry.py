import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import argparse
import numpy as np
import module_kinematics as kin


class OdometryPrinter(Node):
    def __init__(self, topic_name):
        super().__init__('odometry_printer')
        # Subscription to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odometry_callback,
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def odometry_callback(self, msg: Odometry):
        # Extract position
        position = msg.pose.pose.position
        # Extract orientation
        orientation = msg.pose.pose.orientation
        quat = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
        eul = kin.quat_to_eul(quat, deg=True)
        # Extract linear velocity
        linear_velocity = msg.twist.twist.linear
        # Extract angular velocity
        angular_velocity = msg.twist.twist.angular

        # Print extracted values
        self.get_logger().info(
            f"\n"
            f"Position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}\n"
            f"Orientation: roll={eul[0]:.3f}, pitch={eul[1]:.3f}, yaw={eul[2]:.3f}\n"
            f"Linear Velocity: x={linear_velocity.x:.3f}, y={linear_velocity.y:.3f}, z={linear_velocity.z:.3f}\n"
            f"Angular Velocity: x={angular_velocity.x:.3f}, y={angular_velocity.y:.3f}, z={angular_velocity.z:.3f}\n"
        )


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ROS 2 Odometry Printer Node')
    parser.add_argument('--topic', type=str, default='/odom', help='Odometry topic name')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = OdometryPrinter(cli_args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
