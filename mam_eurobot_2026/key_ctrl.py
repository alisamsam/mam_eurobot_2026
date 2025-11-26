#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = (
    "Flèches: ↑ ↓ ← → | ESPACE: stop | q: quitter\n"
    "Vitesses: R/F = lin +/-, T/G = ang +/-, 0 = reset, K = stop d’urgence\n"
    "(debug_keys pour afficher les codes bruts)"
)

def get_key(timeout=0.0):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr:
        return None
    c1 = sys.stdin.read(1)
    if c1 == '\x1b':              # ESC sequence (flèches)
        c2 = sys.stdin.read(1)
        c3 = sys.stdin.read(1)
        return c1 + c2 + c3        # e.g. '\x1b[A' or '\x1bOA'
    return c1

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('v_lin', 0.40)     # m/s par défaut
        self.declare_parameter('v_ang', 1.80)     # rad/s par défaut
        self.declare_parameter('rate', 20.0)      # Hz
        self.declare_parameter('timeout', 0.3)    # s sans touche => stop
        self.declare_parameter('debug_keys', False)

        self.v_lin_min, self.v_lin_max = 0.02, 2.0
        self.v_ang_min, self.v_ang_max = 0.10, 6.0
        self.scale_up, self.scale_dn = 1.10, 1.0/1.10  # +10% / -10%

        topic = self.get_parameter('topic').value
        self.v_lin_default = float(self.get_parameter('v_lin').value)
        self.v_ang_default = float(self.get_parameter('v_ang').value)
        self.v_lin = float(self.v_lin_default)
        self.v_ang = float(self.v_ang_default)
        self.rate  = float(self.get_parameter('rate').value)
        self.timeout_s = float(self.get_parameter('timeout').value)
        self.debug = bool(self.get_parameter('debug_keys').value)

        self.pub = self.create_publisher(Twist, topic, 10)
        self.last_cmd = Twist()
        self.last_key_time = time.time()
        self.timer = self.create_timer(1.0 / max(1.0, self.rate), self.loop)

        self.get_logger().info(HELP)
        self.get_logger().info(f"Publishing to {topic} | Linear vel: {self.v_lin:.2f} m/s, Angular vel: {self.v_ang:.2f} rad/s")

    def loop(self):
        key = get_key(0.0)
        if self.debug and key is not None:
            self.get_logger().info(f"KEY: {repr(key)}")

        tw = Twist()
        speed_changed = False

        if key in ('\x1b[A', '\x1bOA'):  # Up arrow
            tw.linear.x = self.v_lin
            self.last_key_time = time.time()
        elif key in ('\x1b[B', '\x1bOB'):  # Down arrow
            tw.linear.x = -self.v_lin
            self.last_key_time = time.time()
        elif key in ('\x1b[D', '\x1bOD'):  # Left arrow
            tw.angular.z = self.v_ang
            self.last_key_time = time.time()
        elif key in ('\x1b[C', '\x1bOC'):  # Right arrow
            tw.angular.z = -self.v_ang
            self.last_key_time = time.time()
        elif key in ('r','R'):
            self.v_lin = max(self.v_lin_min, min(self.v_lin * self.scale_up, self.v_lin_max))
            speed_changed = True
        elif key in ('f','F'):
            self.v_lin = max(self.v_lin_min, min(self.v_lin * self.scale_dn, self.v_lin_max))
            speed_changed = True
        elif key in ('t','T'):
            self.v_ang = max(self.v_ang_min, min(self.v_ang * self.scale_up, self.v_ang_max))
            speed_changed = True
        elif key in ('g','G'):
            self.v_ang = max(self.v_ang_min, min(self.v_ang * self.scale_dn, self.v_ang_max))
            speed_changed = True
        elif key == '0':
            self.v_lin, self.v_ang = self.v_lin_default, self.v_ang_default
            speed_changed = True
        elif key == ' ':
            self.last_key_time = time.time()
            tw = Twist()  # stop
        elif key in ('k','K'):
            tw = Twist()
            self.pub.publish(tw)
            self.last_cmd = tw
            return
        elif key in ('q', 'Q'):
            self.safe_stop()
            rclpy.shutdown()
            return
        else:
            tw = self.last_cmd

        if time.time() - self.last_key_time > self.timeout_s:
            tw = Twist()  # stop if timeout

        if speed_changed:
            self.get_logger().info(f"Velocities updated: Linear={self.v_lin:.2f} m/s, Angular={self.v_ang:.2f} rad/s")

        self.pub.publish(tw)
        self.last_cmd = tw

    def safe_stop(self):
        self.pub.publish(Twist())

def main():
    use_tty = sys.stdin.isatty()
    settings = None
    try:
        if use_tty:
            settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            print("⚠️  No TTY detected — run via `docker exec -it ... bash` to use arrow keys.")
        rclpy.init()
        node = ArrowTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if use_tty and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            try:
                node.safe_stop()
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

if __name__ == '__main__':
    main()
