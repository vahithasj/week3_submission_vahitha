#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from kratos_vahitha.srv import CalculateArea   # use the service we defined

class AreaClient(Node):
    def __init__(self):
        super().__init__('area_client')
        self.cli = self.create_client(CalculateArea, 'calculate_area')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = CalculateArea.Request()

    def send_request(self, length, width):
        self.req.length = length
        self.req.width = width
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run kratos_vahitha q4_client.py <length> <width>")
        return

    length = float(sys.argv[1])
    width = float(sys.argv[2])

    node = AreaClient()
    response = node.send_request(length, width)
    print(f"✅ Area = {response.area}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

