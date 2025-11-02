#!/usr/bin/env python3
import sys, select, termios, tty, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """\
Flèches : avancer/reculer/tourner | ESPACE: stop | q: quitter
"""

def get_key(timeout=0.0):
    """Lit une touche (y compris flèches) sans bloquer."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr:
        return None
    c1 = sys.stdin.read(1)
    if c1 == '\x1b':                # séquence ESC pour flèches
        c2 = sys.stdin.read(1)
        c3 = sys.stdin.read(1)
        return c1 + c2 + c3
    return c1

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        # paramètres (modifiable via --ros-args -p ...)
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('v_lin', 0.40)   # m/s
        self.declare_parameter('v_ang', 1.80)   # rad/s
        self.declare_parameter('rate', 20.0)    # Hz d’envoi
        self.declare_parameter('timeout', 0.3)  # s sans touche => stop

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.v_lin = float(self.get_parameter('v_lin').value)
        self.v_ang = float(self.get_parameter('v_ang').value)
        self.rate  = float(self.get_parameter('rate').value)
        self.timeout_s = float(self.get_parameter('timeout').value)

        self.pub = self.create_publisher(Twist, topic, 10)
        self.last_cmd = Twist()
        self.last_key_time = time.time()

        period = 1.0 / max(1.0, self.rate)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(HELP)
        self.get_logger().info(f"Pub sur {topic} | v={self.v_lin} m/s, w={self.v_ang} rad/s")

    def loop(self):
        key = get_key(timeout=0.0)
        tw = Twist()

        if key in ('\x1b[A',):         # ↑
            tw.linear.x = self.v_lin
            self.last_key_time = time.time()
        elif key in ('\x1b[B',):       # ↓
            tw.linear.x = -self.v_lin
            self.last_key_time = time.time()
        elif key in ('\x1b[D',):       # ←
            tw.angular.z = self.v_ang
            self.last_key_time = time.time()
        elif key in ('\x1b[C',):       # →
            tw.angular.z = -self.v_ang
            self.last_key_time = time.time()
        elif key == ' ':               # stop immédiat
            self.last_key_time = time.time()
            tw = Twist()
        elif key in ('q', 'Q'):
            self.safe_stop()
            rclpy.shutdown()
            return
        else:
            # aucune nouvelle touche : on décidera via le watchdog
            tw = self.last_cmd

        # 🔒 Watchdog : stop si aucune touche récente
        if time.time() - self.last_key_time > self.timeout_s:
            tw = Twist()

        self.pub.publish(tw)
        self.last_cmd = tw

    def safe_stop(self):
        self.pub.publish(Twist())

def main():
    # terminal en mode non canonique
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        rclpy.init()
        node = ArrowTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # restaure le terminal et stoppe le robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node = ArrowTeleop()  # crée juste pour publier un stop final propre
            node.safe_stop()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
