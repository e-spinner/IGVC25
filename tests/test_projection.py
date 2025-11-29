#!/usr/bin/env python3
"""Simple test for lane_detector.project_pixel_to_ground fallback projection.

This script creates the node with TF disabled and calls project_pixel_to_ground on
center pixel. It asserts the result is not None and z is ~0.
"""
import sys
import time
import math
import os

# ensure src on path
ROOT = os.path.dirname(os.path.dirname(__file__))
src_path = os.path.join(ROOT, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import rclpy
from igvc.igvc_vision.lane_detector import LaneDetectorNode


def run_test():
    rclpy.init()
    node = LaneDetectorNode()
    # set params to use fallback extrinsics
    node.set_parameters([rclpy.parameter.Parameter('use_tf', rclpy.Parameter.Type.BOOL, False),
                         rclpy.parameter.Parameter('camera_height', rclpy.Parameter.Type.DOUBLE, 0.35),
                         rclpy.parameter.Parameter('camera_pitch_deg', rclpy.Parameter.Type.DOUBLE, -10.0)])

    # use fallback intrinsics matching lane_detector default
    K = [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
    u = 320.0
    v = 240.0

    pt = node.project_pixel_to_ground(u, v, __import__('numpy').array(K), 'test_cam_frame', None)
    node.get_logger().info(f'Projection result for center pixel: {pt}')

    if pt is None:
        print('TEST FAIL: projection returned None')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    x, y, z = pt
    if abs(z) > 1e-2:
        print(f'TEST FAIL: projected z not near 0 (z={z})')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(3)

    # forward distance must be positive
    if x <= 0 and y == 0:
        print(f'TEST FAIL: projected point not in front of camera: {pt}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(4)

    print('TEST PASS: projection produced ground point', pt)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_test()
