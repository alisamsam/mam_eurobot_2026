#!/usr/bin/env python3

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


DEFAULT_WAYPOINTS = [
    0.2, 0.0, 0.0,
    0.6, 0.0, 0.0,
    0.6, 0.4, 1.57,
    0.2, 0.4, 3.14,
]


class MainStrategy(Node):
    """Execute une trajectoire predefinie de 4 points avec Nav2."""

    def __init__(self):
        super().__init__("main_strategy")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("pause_sec", 0.0)
        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)

        self.frame_id = self.get_parameter("frame_id").value
        self.action_name = self.get_parameter("action_name").value
        self.pause_sec = float(self.get_parameter("pause_sec").value)
        self.waypoints = self._parse_waypoints(self.get_parameter("waypoints").value)

        self.client = ActionClient(self, NavigateToPose, self.action_name)

    def _parse_waypoints(self, raw_values):
        try:
            values = [float(v) for v in raw_values]
        except (TypeError, ValueError):
            self.get_logger().warn(
                "Parametre 'waypoints' invalide, utilisation des valeurs par defaut."
            )
            values = DEFAULT_WAYPOINTS[:]

        if len(values) < 12 or len(values) % 3 != 0:
            self.get_logger().warn(
                "Le parametre 'waypoints' doit contenir 4 points (x,y,yaw). "
                "Utilisation des valeurs par defaut."
            )
            values = DEFAULT_WAYPOINTS[:]

        points = []
        for idx in range(0, min(len(values), 12), 3):
            x, y, yaw = values[idx:idx + 3]
            points.append((x, y, yaw))
        return points

    def _pose_from_xy_yaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def run(self):
        self.get_logger().info("Attente du serveur NavigateToPose...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Le serveur NavigateToPose n'est pas disponible.")
            return

        for idx, (x, y, yaw) in enumerate(self.waypoints, start=1):
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_from_xy_yaw(x, y, yaw)
            self.get_logger().info(
                f"Envoi du goal {idx}/4: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )

            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Goal refuse par Nav2.")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                status = "UNKNOWN" if result is None else str(result.status)
                self.get_logger().error(f"Goal {idx} echoue (status={status}).")
                return

            self.get_logger().info(f"Goal {idx} atteint.")
            if self.pause_sec > 0.0:
                time.sleep(self.pause_sec)

        self.get_logger().info("Trajet termine.")


def main(args=None):
    rclpy.init(args=args)
    node = MainStrategy()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
