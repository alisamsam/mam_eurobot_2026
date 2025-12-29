#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class OpenCVCam(Node):
    def __init__(self):
        super().__init__('open_cv_cam')

        # Paramètre dynamique : topic image
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # QoS adapté aux caméras simulées (reliable, volatile)
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_count = 0
        self.t0 = self.get_clock().now()

        self.sub_img = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.get_logger().info(f"📷 Subscribed to: {image_topic}")

        self.timer = self.create_timer(1.0, self._log_rate)

        # Essai d’ouverture de la fenêtre OpenCV
        try:
            cv2.namedWindow('OpenCV Camera', cv2.WINDOW_NORMAL)
            self.use_window = True
        except Exception as e:
            self.get_logger().warn(f'OpenCV window creation failed (headless env?): {e}')
            self.use_window = False

    def on_image(self, msg: Image):
        if self.frame_count == 0:
            self.get_logger().info(
                f'Image received: {msg.width}x{msg.height}, encoding={msg.encoding}'
            )

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        if self.use_window:
            cv2.imshow('OpenCV Camera', cv_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Shutting down node...")
                rclpy.shutdown()
        else:
            # Mode sans fenêtre : sauvegarde périodique
            if self.frame_count % 60 == 0:
                cv2.imwrite('/tmp/opencv_cam_sample.jpg', cv_img)

        self.frame_count += 1

    def _log_rate(self):
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        inc = self.frame_count - self.last_count
        hz = inc / dt if dt > 0 else 0.0
        self.get_logger().info(f"Frame rate ≈ {hz:.2f} Hz (+{inc} frames in {dt:.2f}s)")
        self.t0 = now
        self.last_count = self.frame_count
        if inc == 0:
            self.get_logger().warn('⚠️ No new frames received in the last second.')


def main():
    rclpy.init()
    node = OpenCVCam()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
