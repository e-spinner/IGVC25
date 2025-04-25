#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ScanRepublisher(Node):
    def __init__(self):
        super().__init__('scan_republisher')

        qos_sub = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_pub = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(LaserScan, 'scan_be', self.callback, qos_sub)
        self.pub = self.create_publisher(LaserScan, 'scan', qos_pub)

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()