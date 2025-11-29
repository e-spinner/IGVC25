#!/usr/bin/env python3
"""Additional projection tests for lane_detector.project_pixel_to_ground.

Runs a few cases with the fallback projection (use_tf=False) to validate
center/off-center pixels and some edge cases.

Exits with code 0 on full pass; non-zero otherwise.
"""
import sys
import os
import math
import numpy as np

# ensure src on path
ROOT = os.path.dirname(os.path.dirname(__file__))
src_path = os.path.join(ROOT, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import rclpy
from igvc.igvc_vision.lane_detector import LaneDetectorNode


def approx_zero(x, tol=1e-2):
    return abs(x) <= tol


def main():
    rclpy.init()
    node = LaneDetectorNode()

    # use fallback extrinsics
    node.set_parameters([
        rclpy.parameter.Parameter('use_tf', rclpy.Parameter.Type.BOOL, False),
        rclpy.parameter.Parameter('camera_height', rclpy.Parameter.Type.DOUBLE, 0.35),
        rclpy.parameter.Parameter('camera_pitch_deg', rclpy.Parameter.Type.DOUBLE, -10.0),
    ])

    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])

    failures = 0

    # Test 1: center pixel
    u, v = 320.0, 240.0
    p = node.project_pixel_to_ground(u, v, K, 'test_cam', None)
    node.get_logger().info(f"ExtraTest center -> {p}")
    if p is None:
        print('FAIL: center projection returned None')
        failures += 1
    else:
        x, y, z = p
        if not approx_zero(z):
            print(f'FAIL: center z not ~0 (z={z})')
            failures += 1
        if x <= 0:
            print(f'FAIL: center point not in front (x={x})')
            failures += 1

    # Test 2: off-center pixel
    u, v = 100.0, 200.0
    p = node.project_pixel_to_ground(u, v, K, 'test_cam', None)
    node.get_logger().info(f"ExtraTest off-center -> {p}")
    if p is None:
        print('FAIL: off-center projection returned None')
        failures += 1

    # Test 3: extremely pitched camera up (should often return None because ray points up)
    node.set_parameters([
        rclpy.parameter.Parameter('camera_pitch_deg', rclpy.Parameter.Type.DOUBLE, 80.0),
    ])
    p = node.project_pixel_to_ground(320.0, 240.0, K, 'test_cam', None)
    node.get_logger().info(f"ExtraTest up-pitch -> {p}")
    if p is not None:
        print(f'FAIL: up-pitch expected None but got {p}')
        failures += 1

    # Test 4: very low camera height (should still project but very close)
    node.set_parameters([
        rclpy.parameter.Parameter('camera_pitch_deg', rclpy.Parameter.Type.DOUBLE, -10.0),
        rclpy.parameter.Parameter('camera_height', rclpy.Parameter.Type.DOUBLE, 0.05),
    ])
    p = node.project_pixel_to_ground(320.0, 240.0, K, 'test_cam', None)
    node.get_logger().info(f"ExtraTest low-height -> {p}")
    if p is None:
        print('FAIL: low-height center projection returned None')
        failures += 1

    if failures == 0:
        print('ALL EXTRA TESTS PASS')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    else:
        print(f'{failures} FAILURES')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
