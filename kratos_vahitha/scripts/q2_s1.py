#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class S1Publisher(Node):
    def __init__(self):
        super().__init__('s1_publisher')
        self.publisher_ = self.create_publisher(String, '/s1', 10)
        self.timer = self.create_timer(1.0, self.publish_signal)  # check every 1 sec
        self.start_time = time.time()
        self.state = "green"

    def publish_signal(self):
        elapsed = int(time.time() - self.start_time)
        # Change state every 10 seconds
        if elapsed % 20 < 10:
            self.state = "green"
        else:
            self.state = "red"

        msg = String()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'S1 publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = S1Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

