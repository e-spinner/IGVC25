#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from igvc25.msg import RootInstruction


class RootController(Node):
  def __init__(self):
    super().__init__("root_controller")
    self.publisher_ = self.create_publisher(RootInstruction, "/root_instr", 10)
    self.timer = self.create_timer(1, self.publish_instruction)
    self.count = 0

  def publish_instruction(self):
    msg = RootInstruction()
    msg.left_speed = 10.0
    msg.right_speed = 10.0
    msg.duration = 1.0
    self.publisher_.publish(msg)
    self.get_logger().info(f"Published instruction #{self.count}")
    self.count += 1


def main(args=None):
  rclpy.init(args=args)
  node = RootController()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
