#!/usr/bin/env python3
import os
import json
import asyncio
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Root, event
from irobot_edu_sdk.music import Note

from igvc25.msg import RootInstruction


class RootController(Node):
  def __init__(self, robot: Root, loop):
    super().__init__("root_controller")

    self.publisher = self.create_publisher(RootInstruction, "/root_instr", 10)

    self.robot = robot
    self.loop = loop
    self.robot_ready = False

    self.timer = self.create_timer(1, self.timer_callback)
    self.count = 0

    self.get_logger().info("Root Controller Node initialized")

  def timer_callback(self):
    if self.robot_ready:
      # publish intruction
      msg = RootInstruction()
      msg.left_speed = 10.0
      msg.right_speed = 10.0
      msg.duration = 1.0

      self.publisher.publish(msg)
      self.get_logger().info(
        f"Published command #{self.count}: L={msg.left_speed}, R={msg.right_speed}, D={msg.duration}"
      )

      # Send the same command to the Root robot
      # Schedule the robot command as a task
      asyncio.run_coroutine_threadsafe(
        self.send_robot_command(msg.left_speed, msg.right_speed, msg.duration),
        self.loop,
      )

      self.count += 1

    else:
      self.get_logger().warn("Robot not ready yet, skipping robot command")

  async def send_robot_command(self, left_speed, right_speed, duration):
    try:
      await self.robot.set_wheel_speeds(left_speed, right_speed)

      await asyncio.sleep(duration)

    except Exception as e:
      self.get_logger().error(f"Error sending command to robot: {e}")


def ros_spin_thread(node, executor: SingleThreadedExecutor):
  try:
    executor.spin()
  except Exception as e:
    print(f"ROS spin error: {e}")


def main(args=None):
  rclpy.init()

  robot = Root(Bluetooth("wesbo"))
  loop = asyncio.get_event_loop()

  node = RootController(robot, loop)

  executor = SingleThreadedExecutor()
  executor.add_node(node)

  ros_thread = Thread(
    target=ros_spin_thread, args=(node, executor), daemon=True
  )
  ros_thread.start()

  @event(robot.when_play)
  async def play(robot):
    await robot.set_lights_on_rgb(128, 0, 255)
    await robot.play_note(Note.A5, 0.5)
    node.robot_ready = True
    node.get_logger().info("Robot started!")

  try:
    node.get_logger().info("Starting robot connection...")
    robot.play()
  except KeyboardInterrupt:
    node.get_logger().info("Shutting down...")
  finally:
    # Cleanup
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)


if __name__ == "__main__":
  main()
