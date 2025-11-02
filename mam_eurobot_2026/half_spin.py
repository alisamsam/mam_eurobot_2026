#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math

class Turn180Node(Node):
    def __init__(self):
        super().__init__("turn_180_node")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.elapsed_time = 0.0

        # Paramètres
        self.angular_speed = 1.57  # rad/s (≈ 90°/s)
        self.target_angle = math.pi  # 180° en radians
        self.duration = self.target_angle / self.angular_speed

        self.get_logger().info(f"Rotating robot 180° in {self.duration:.2f} seconds")

    def timer_callback(self):
        msg = Twist()
        if self.elapsed_time < self.duration:
            msg.angular.z = self.angular_speed
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
            self.elapsed_time += self.timer_period
            self.get_logger().info(f"Turning... ({self.elapsed_time:.1f}/{self.duration:.1f}s)")
        else:
            # Stop the robot
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("Rotation complete. Stopping robot.")
            self.timer.cancel()
            self.create_timer(0.1, self.shutdown_node)

    def shutdown_node(self):
        self.get_logger().info("Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Turn180Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
