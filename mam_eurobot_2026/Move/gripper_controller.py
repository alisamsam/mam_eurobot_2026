#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller")

        self.declare_parameter("command_topic", "/gripper/command")
        self.declare_parameter("open_position", 0.04)
        self.declare_parameter("closed_position", 0.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("start_open", True)

        self.command_topic = self.get_parameter("command_topic").value
        self.open_position = float(self.get_parameter("open_position").value)
        self.closed_position = float(self.get_parameter("closed_position").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        start_open = bool(self.get_parameter("start_open").value)

        if publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz <= 0, utilisation de 10.0 Hz.")
            publish_rate_hz = 10.0

        self.target_position = self.open_position if start_open else self.closed_position

        self.publisher = self.create_publisher(Float64, self.command_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_target)
        self.service = self.create_service(SetBool, "gripper/set_open", self._handle_set_open)

        self.get_logger().info(
            f"GripperController prêt. Topic={self.command_topic}, open={self.open_position}, "
            f"closed={self.closed_position}, rate={publish_rate_hz} Hz"
        )

    def _handle_set_open(self, request, response):
        self.target_position = self.open_position if request.data else self.closed_position
        state = "open" if request.data else "closed"
        response.success = True
        response.message = f"Gripper set to {state} ({self.target_position})."
        return response

    def _publish_target(self):
        msg = Float64()
        msg.data = float(self.target_position)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
