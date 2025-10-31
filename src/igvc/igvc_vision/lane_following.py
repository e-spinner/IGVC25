"""Simple lane following ROS2 node using OpenCV.

Subscribes to a camera image topic (default `/left_camera/image_raw`),
detects lane lines with a classical CV pipeline, computes a steering
correction, and publishes a Twist on `/cmd_vel`.

This is an initial implementation intended to be iterated on. It
publishes a debug image on `/lane_follow/debug_image` (tunable).
"""
from __future__ import annotations

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__("lane_following")

        # Parameters (tunable)
        self.declare_parameter("image_topic", "/left_camera/image_raw")
    # publish into the twist_mux navigation input so multiplexing works in sim
    self.declare_parameter("cmd_topic", "/cmd_vel_nav_smoothed")
        self.declare_parameter("debug_image_topic", "/lane_follow/debug_image")
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("steer_gain", 0.003)  # rad per pixel error
        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("hough_rho", 1)
        self.declare_parameter("hough_theta", np.pi / 180)
        self.declare_parameter("hough_threshold", 20)
        self.declare_parameter("hough_min_line_len", 20)
        self.declare_parameter("hough_max_line_gap", 30)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.debug_image_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.steer_gain = self.get_parameter("steer_gain").get_parameter_value().double_value

        # OpenCV params
        self.canny_low = int(self.get_parameter("canny_low").get_parameter_value().integer_value)
        self.canny_high = int(self.get_parameter("canny_high").get_parameter_value().integer_value)
        self.hough_rho = float(self.get_parameter("hough_rho").get_parameter_value().double_value)
        self.hough_theta = float(self.get_parameter("hough_theta").get_parameter_value().double_value)
        self.hough_threshold = int(self.get_parameter("hough_threshold").get_parameter_value().integer_value)
        self.hough_min_line_len = int(self.get_parameter("hough_min_line_len").get_parameter_value().integer_value)
        self.hough_max_line_gap = int(self.get_parameter("hough_max_line_gap").get_parameter_value().integer_value)

        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 1)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)

        self.get_logger().info(f"LaneFollowing: subscribing to {self.image_topic}")
        self.get_logger().info(f"LaneFollowing: publishing cmd to {self.cmd_topic}")

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        steering_angle, debug_img = self.process_image(cv_image)

        # Map steering angle to Twist (simple differential)
        twist = Twist()

        # forward speed scaled by confidence (here we keep constant forward speed when we have a steering angle)
        if steering_angle is None:
            # no detection: slow/stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn_once("No lane detected — stopping").
        else:
            twist.linear.x = float(self.max_linear_speed)
            # clamp angular speed
            twist.angular.z = float(max(-self.max_angular_speed, min(self.max_angular_speed, steering_angle)))

        self.cmd_pub.publish(twist)

        # publish debug image
        try:
            if debug_img is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge publish error: {e}")

    def process_image(self, img: np.ndarray) -> tuple[float | None, np.ndarray | None]:
        """Very small lane detection pipeline.

        Returns (steering_angle_rad, debug_image).
        steering_angle is positive for left turn (counter-clockwise angular z).
        """
        h, w = img.shape[:2]

        # resize for faster processing (keep aspect ratio)
        target_w = 640
        scale = target_w / float(w)
        img_r = cv2.resize(img, (target_w, int(h * scale)), interpolation=cv2.INTER_AREA)
        h_r, w_r = img_r.shape[:2]

        # ROI: lower half
        roi = img_r[int(h_r * 0.5) : h_r, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)

        # mask central region to focus on lanes
        mask = np.zeros_like(edges)
        polygon = np.array([
            [int(0.0 * w_r), int(1.0 * roi.shape[0])],
            [int(0.0 * w_r), int(0.0 * roi.shape[0])],
            [int(1.0 * w_r), int(0.0 * roi.shape[0])],
            [int(1.0 * w_r), int(1.0 * roi.shape[0])],
        ])
        cv2.fillPoly(mask, [polygon], 255)
        masked = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(
            masked,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_len,
            maxLineGap=self.hough_max_line_gap,
        )

        debug_img = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR)

        if lines is None:
            return None, debug_img

        # separate lines into left and right by slope
        left_lines = []
        right_lines = []
        for l in lines:
            x1, y1, x2, y2 = l[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / float(x2 - x1)
            # discard near-horizontal
            if abs(slope) < 0.3:
                continue
            if slope < 0:
                left_lines.append(l[0])
            else:
                right_lines.append(l[0])

        def average_line(lines_arr):
            if len(lines_arr) == 0:
                return None
            xs = []
            ys = []
            for x1, y1, x2, y2 in lines_arr:
                xs += [x1, x2]
                ys += [y1, y2]
            if len(xs) < 2:
                return None
            poly = np.polyfit(xs, ys, 1)  # y = m x + b
            m = poly[0]
            b = poly[1]
            # convert to two points spanning ROI height
            y1 = roi.shape[0]
            y2 = int(roi.shape[0] * 0.4)
            x1 = int((y1 - b) / m)
            x2 = int((y2 - b) / m)
            return (x1, y1, x2, y2)

        left_avg = average_line(left_lines)
        right_avg = average_line(right_lines)

        # draw lines
        if left_avg is not None:
            cv2.line(debug_img, (left_avg[0], left_avg[1]), (left_avg[2], left_avg[3]), (0, 0, 255), 3)
        if right_avg is not None:
            cv2.line(debug_img, (right_avg[0], right_avg[1]), (right_avg[2], right_avg[3]), (0, 255, 0), 3)

        # compute vanishing point as intersection of the two averaged lines
        steering_angle = None
        if left_avg is not None and right_avg is not None:
            # Convert lines to slope-intercept for full image coordinates
            def line_params(x1, y1, x2, y2):
                if x2 == x1:
                    return None
                m = (y2 - y1) / float(x2 - x1)
                b = y1 - m * x1
                return m, b

            m1, b1 = line_params(*left_avg)
            m2, b2 = line_params(*right_avg)
            if m1 is not None and m2 is not None and abs(m1 - m2) > 1e-3:
                # x_intersect = (b2 - b1) / (m1 - m2)
                x_vp = (b2 - b1) / (m1 - m2)
                # compute pixel error relative to image center
                center_x = w_r / 2.0
                error_px = x_vp - center_x
                steering_angle = -float(error_px) * self.steer_gain

                # draw VP
                cv2.circle(debug_img, (int(x_vp), int(roi.shape[0] * 0.5)), 6, (255, 0, 0), -1)

        return steering_angle, debug_img


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
