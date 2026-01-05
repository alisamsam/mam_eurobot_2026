#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('test_movement_node')

    publisher = node.create_publisher(Twist, '/mecanum_drive_controller/cmd_vel_unstamped', 10)

    twist = Twist()
    twist.linear.x = 0.5
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    # Give some time for the publisher to connect
    time.sleep(10)

    node.get_logger().info("Publishing a test command to move the robot forward.")
    publisher.publish(twist)

    # Allow some time for the message to be processed
    time.sleep(10)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
