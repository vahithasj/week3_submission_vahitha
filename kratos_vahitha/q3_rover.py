#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from builtin_interfaces.msg import Time
from kratos_vahitha.msg import RoverData

import time

class RoverPublisher(Node):
    def __init__(self):
        super().__init__('rover_publisher')
        self.publisher_ = self.create_publisher(RoverData, '/rover_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # every second
        self.start_time = time.time()
        self.distance = 0.0

    def publish_data(self):
        msg = RoverData()

        # 1. Velocity
        msg.velocity = Twist()
        msg.velocity.linear.x = 1.0
        msg.velocity.angular.z = 0.1

        # 2. Distance (just increasing)
        self.distance += 1.0
        msg.distance_traveled = self.distance

        # 3. Coordinates (use distance as x)
        msg.coordinates = Pose()
        msg.coordinates.position.x = self.distance
        msg.coordinates.position.y = 0.0
        msg.coordinates.position.z = 0.0

        # 4. Battery (decreasing)
        msg.battery_level = max(0.0, 100.0 - self.distance * 0.5)

        # 5. Travel time
        elapsed = int(time.time() - self.start_time)
        msg.travel_time = Time(sec=elapsed)

        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published Rover Data | Distance: {msg.distance_traveled}, Battery: {msg.battery_level:.2f}%'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RoverPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

