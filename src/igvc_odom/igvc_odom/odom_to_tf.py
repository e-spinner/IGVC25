#!/usr/bin/env python3
"""Subscribe to nav_msgs/Odometry and broadcast odom -> child_frame_id on /tf."""
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.declare_parameter("odom_topic", "odom")
        topic = str(self.get_parameter("odom_topic").value)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self._on_odom, 50)
        self.get_logger().info('Broadcasting TF from "%s" odometry messages', topic)

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
