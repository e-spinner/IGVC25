#!/usr/bin/env python3
"""Publish synthetic camera images and CameraInfo to feed the lane detector.
Publishes to /left_camera/image_raw and /left_camera/camera_info at ~10Hz for 5s.
"""
import sys
import os
import time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(__file__))
src_path = os.path.join(ROOT, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

import rclpy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge


def main():
    rclpy.init()
    node = rclpy.create_node('sim_camera_pub')
    img_pub = node.create_publisher(Image, '/left_camera/image_raw', 10)
    info_pub = node.create_publisher(CameraInfo, '/left_camera/camera_info', 1)
    bridge = CvBridge()

    # Create a simple synthetic image (gray gradient)
    h, w = 480, 640
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for i in range(h):
        img[i, :, :] = int(255 * (i / h))

    cam_info = CameraInfo()
    cam_info.width = w
    cam_info.height = h
    cam_info.k = [500.0, 0.0, w / 2.0, 0.0, 500.0, h / 2.0, 0.0, 0.0, 1.0]
    cam_info.header.frame_id = 'left_camera'

    start = time.time()
    rate = 10.0
    count = 0
    try:
        while time.time() - start < 5.0:
            hdr = Header()
            hdr.stamp = node.get_clock().now().to_msg()
            hdr.frame_id = 'left_camera'

            img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header = hdr
            cam_info.header = hdr

            img_pub.publish(img_msg)
            info_pub.publish(cam_info)
            count += 1
            time.sleep(1.0 / rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Published {count} images')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
