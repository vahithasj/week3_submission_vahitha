#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RoverSequence(Node):
    def __init__(self):
        super().__init__('rover_sequence')
        self.pub = self.create_publisher(Float64MultiArray, '/rover/control_input', 10)
        self.rate = 0.1  # 10 Hz

    def run_step(self, data, duration):
        msg = Float64MultiArray()
        msg.data = data
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self.pub.publish(msg)
            time.sleep(self.rate)
        self.pub.publish(Float64MultiArray(data=[0.0]*8))
        time.sleep(0.2)

def main():
    rclpy.init()
    node = RoverSequence()
    try:
        node.run_step([0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0], 2.0)  # forward 2m
        node.run_step([-math.pi/2]*4 + [0.0,0.0,0.0,0.0], 1.0)  # steer -90
        node.run_step([-math.pi/2]*4 + [1.0,1.0,1.0,1.0], 2.0)  # sideways
        node.run_step([0.0,0.0,0.0,0.0,-1.0,-1.0,1.0,1.0], 2.0)  # diff rotate
        node.run_step([2.35619,0.785398,-2.35619,-0.785398,1.0,1.0,1.0,1.0], 2.0)  # independent rotate
        node.run_step([math.pi/4]*4 + [1.0,1.0,1.0,1.0], 2.0)  # diagonal
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

