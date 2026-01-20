import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class LidarScanRelay(Node):
    def __init__(self):
        super().__init__('lidar_scan_relay')
        qos_in = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_out = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(
            LaserScan,
            '/livox/scan_best_effort',
            self.relay_callback,
            qos_in
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/livox/scan_reliable',
            qos_out
        )

    def relay_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarScanRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
