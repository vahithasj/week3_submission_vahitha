#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class S2Node(Node):
    def __init__(self):
        super().__init__('s2_node')
        self.subscription = self.create_subscription(
            String,
            '/s1',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/s2', 10)

    def listener_callback(self, msg):
        # Opposite of S1
        if msg.data == "green":
            s2_state = "red"
        else:
            s2_state = "green"

        out_msg = String()
        out_msg.data = s2_state
        self.publisher_.publish(out_msg)
        self.get_logger().info(f'S2 publishing: "{out_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = S2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

