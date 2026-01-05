#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class AutoMove(Node):
    """Fait avancer le robot puis le fait tourner pour une courte routine autonome."""

    def __init__(self):
        super().__init__("auto_move")

        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 1.40)
        self.declare_parameter("angular_speed", 2.0)
        self.declare_parameter("forward_duration", 15.0)  # secondes
        self.declare_parameter("turn_duration", 10.0)      # secondes
        self.declare_parameter("rate", 1.0)              # Hz

        self.topic = self.get_parameter("topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.forward_duration = float(self.get_parameter("forward_duration").value)
        self.turn_duration = float(self.get_parameter("turn_duration").value)
        self.dt = 1.0 / max(1.0, float(self.get_parameter("rate").value))

        self.publisher_ = self.create_publisher(Twist, self.topic, 10)
        self.sequence_start = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self._tick)
        self.shutdown_timer = None

        self.get_logger().info(
            f"AutoMove lancé: avance {self.forward_duration}s à {self.linear_speed:.2f} m/s, "
            f"puis tourne {self.turn_duration}s à {self.angular_speed:.2f} rad/s sur {self.topic}"
        )

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.sequence_start).nanoseconds / 1e9
        msg = Twist()

        if elapsed < self.forward_duration:
            msg.linear.x = self.linear_speed
            self.publisher_.publish(msg)
            return

        if elapsed < self.forward_duration + self.turn_duration:
            msg.angular.z = self.angular_speed
            self.publisher_.publish(msg)
            return

        # Routine terminée: stop et fermeture du noeud.
        self.publisher_.publish(Twist())
        self.get_logger().info("Séquence terminée: robot à l'arrêt.")
        self.timer.cancel()
        if self.shutdown_timer is None:
            self.shutdown_timer = self.create_timer(0.1, self._shutdown)

    def _shutdown(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None
        self.publisher_.publish(Twist())
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
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
