#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import threading
import sys

L1 = 2.0
L2 = 1.5
MAX_STEP = 0.5

class Q3IKNode(Node):
    def __init__(self):
        super().__init__('q3_ik_node')

        # current end-effector position
        self.current_x = 0.0
        self.current_y = 0.0

        # subscriber
        self.sub = self.create_subscription(
            Point,
            '/end_effector_position',
            self.position_callback,
            10)

        # publisher
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_angles_goal',
            10)

        self.get_logger().info("Q3 IK Node started ✅")

        # start user input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

    def position_callback(self, msg: Point):
        self.current_x = msg.x
        self.current_y = msg.y

    def user_input_loop(self):
        while self.running and rclpy.ok():
            try:
                sys.stdout.write("\nDirection (x/y, q to quit): ")
                sys.stdout.flush()
                direction = sys.stdin.readline().strip().lower()
                if direction == 'q':
                    self.get_logger().info("Exiting node.")
                    rclpy.shutdown()
                    break
                if direction not in ['x','y']:
                    self.get_logger().warn("Invalid direction.")
                    continue

                sys.stdout.write("Distance (<=0.5): ")
                sys.stdout.flush()
                step = float(sys.stdin.readline().strip())
                if abs(step) > MAX_STEP:
                    self.get_logger().warn("Step too large!")
                    continue

                target_x = self.current_x + (step if direction=='x' else 0.0)
                target_y = self.current_y + (step if direction=='y' else 0.0)

                reachable, sols = self.inverse_kinematics(target_x, target_y)
                if not reachable:
                    self.get_logger().warn(f"Target ({target_x:.2f},{target_y:.2f}) unreachable ❌")
                    continue

                theta1, theta2 = sols[0]  # elbow-down solution
                msg = Float64MultiArray()
                msg.data = [theta1, theta2]
                self.pub.publish(msg)

                self.get_logger().info(
                    f"Target ({target_x:.2f},{target_y:.2f}) -> θ1={theta1:.3f}, θ2={theta2:.3f}"
                )

            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def inverse_kinematics(self, x, y):
        r = math.hypot(x, y)
        rmin = abs(L1 - L2)
        rmax = L1 + L2
        if not (rmin <= r <= rmax):
            return False, []

        cos_t2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))
        t2a = math.acos(cos_t2)
        t2b = -t2a

        sols = []
        for t2 in (t2a, t2b):
            k1 = L1 + L2*math.cos(t2)
            k2 = L2*math.sin(t2)
            t1 = math.atan2(y, x) - math.atan2(k2, k1)
            sols.append((t1, t2))
        return True, sols

def main(args=None):
    rclpy.init(args=args)
    node = Q3IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

