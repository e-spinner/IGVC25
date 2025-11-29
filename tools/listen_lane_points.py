#!/usr/bin/env python3
"""Subscribe to /lane_points and print out the first message received.
Exits after printing the header and basic metadata.
"""
import sys
import os

ROOT = os.path.dirname(os.path.dirname(__file__))
src_path = os.path.join(ROOT, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import rclpy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('lane_points_listener')

    got = {'flag': False}

    def cb(msg: PointCloud2):
        node.get_logger().info(f'Received PointCloud2 header.frame_id={msg.header.frame_id} width={msg.width} height={msg.height} fields={[f.name for f in msg.fields]}')
        node.get_logger().info(f'data length={len(msg.data)}')
        got['flag'] = True

    node.create_subscription(PointCloud2, '/lane_points', cb, 10)

    # wait up to timeout seconds for a message
    timeout = 8.0
    start = node.get_clock().now().nanoseconds / 1e9
    try:
        while not got['flag'] and (node.get_clock().now().nanoseconds / 1e9 - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    if not got['flag']:
        node.get_logger().warn('No /lane_points message received within timeout')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
