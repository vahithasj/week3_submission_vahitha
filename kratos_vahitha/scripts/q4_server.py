#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kratos_vahitha.srv import CalculateArea   # <-- use our service

class AreaServer(Node):
    def __init__(self):
        super().__init__('area_server')
        self.srv = self.create_service(CalculateArea, 'calculate_area', self.calculate_area_callback)
        self.get_logger().info("✅ Area Server is ready...")

    def calculate_area_callback(self, request, response):
        response.area = request.length * request.width
        self.get_logger().info(
            f"Received: length={request.length}, width={request.width} → area={response.area}"
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AreaServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

