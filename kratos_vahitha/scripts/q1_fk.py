#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.get_logger().info("Q1 Forward Kinematics node is running 🚀")

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

