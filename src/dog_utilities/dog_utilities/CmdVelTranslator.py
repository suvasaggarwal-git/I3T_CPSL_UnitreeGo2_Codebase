import os
import sys
import socket
import json
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

SPORT_SOCKET   = "/tmp/go2_sport.sock"
BRIDGE_SCRIPT  = os.path.join(os.path.dirname(__file__),
                               '..', 'scripts', 'sport_bridge.py')
BRIDGE_TIMEOUT = 15.0   # seconds to wait for bridge socket to appear


def _start_bridge():
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = (
        '/home/unitree/cyclonedds_ws/install/cyclonedds/lib:'
        + env.get('LD_LIBRARY_PATH', '')
    )
    env['CYCLONEDDS_URI'] = 'file:///home/unitree/cyclonedds_ws/cyclonedds.xml'
    return subprocess.Popen(
        [sys.executable, os.path.realpath(BRIDGE_SCRIPT)],
        env=env,
    )


class CmdVelTranslator(Node):
    def __init__(self, bridge_proc):
        super().__init__('cmd_vel_translator')
        self._bridge = bridge_proc
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, qos)
        self.get_logger().info(f"CmdVelTranslator ready — forwarding to {SPORT_SOCKET}")

    def _cmd_vel_cb(self, msg: Twist):
        payload = json.dumps({
            "vx":   round(msg.linear.x,  2),
            "vy":   round(msg.linear.y,  2),
            "vyaw": round(msg.angular.z, 2),
        }).encode()
        try:
            self._sock.sendto(payload, SPORT_SOCKET)
        except Exception as e:
            self.get_logger().error(
                f"cmd_vel send failed: {e}",
                throttle_duration_sec=2.0
            )

    def destroy_node(self):
        if self._bridge and self._bridge.poll() is None:
            self._bridge.terminate()
        super().destroy_node()


def main(args=None):
    # Start bridge subprocess before initialising ROS
    bridge = _start_bridge()

    # Wait for the socket to appear
    deadline = time.monotonic() + BRIDGE_TIMEOUT
    while not os.path.exists(SPORT_SOCKET):
        if time.monotonic() > deadline:
            print(f"[CmdVelTranslator] ERROR: sport_bridge socket never appeared "
                  f"(bridge exit code: {bridge.poll()})", flush=True)
            bridge.terminate()
            return
        time.sleep(0.2)

    rclpy.init(args=args)
    node = CmdVelTranslator(bridge)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
