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

    self.declare_parameter("commands_file", "commands.json")

    commands_file = (
      self.get_parameter("commands_file").get_parameter_value().string_value
    )

    self.commands = self.load_commands(commands_file)
    self.command_index = 0

    self.timer = None
    self.count = 0

    self.get_logger().info("Root Controller Node initialized")

  def load_commands(self, filepath: str):
    try:
      with open(filepath, "r") as f:
        data = json.load(f)
        commands = data.get("commands", [])
        self.get_logger().info(
          f"Loaded {len(commands)} commands from {filepath}"
        )
        return commands
    except FileNotFoundError:
      self.get_logger().error(f"Command file not found: {filepath}")
      return []
    except json.JSONDecodeError as e:
      self.get_logger().error(f"Error parsing JSON file: {e}")
      return []

  def start_execution(self):
    if len(self.commands) == 0:
      self.get_logger().warn("No commands to execute")
      return

    self.get_logger().info("Starting command execution...")
    self.execute_next_command()

  def execute_next_command(self):
    if self.command_index >= len(self.commands):
      self.get_logger().info("All commands completed!")
      return

    cmd = self.commands[self.command_index]

    # Create and publish ROS message
    msg = RootInstruction()
    msg.left_speed = float(cmd.get("left_speed", 0.0))
    msg.right_speed = float(cmd.get("right_speed", 0.0))
    msg.duration = float(cmd.get("duration", 1.0))

    self.publisher.publish(msg)
    self.get_logger().info(
      f"Command {self.command_index + 1}/{len(self.commands)}: "
      f"L={msg.left_speed}, R={msg.right_speed}, D={msg.duration}"
    )

    # Send command to robot
    asyncio.run_coroutine_threadsafe(
      self.send_robot_command(msg.left_speed, msg.right_speed, msg.duration),
      self.loop,
    )

    self.command_index += 1

    # Schedule next command after duration
    if self.timer:
      self.timer.cancel()
    self.timer = self.create_timer(msg.duration, self.execute_next_command)

  async def send_robot_command(self, left_speed, right_speed, duration):
    try:
      await self.robot.set_wheel_speeds(left_speed, right_speed)

      await asyncio.sleep(duration)

      await self.robot.set_wheel_speeds(0, 0)

    except Exception as e:
      self.get_logger().error(f"Error sending command to robot: {e}")


def ros_spin_thread(node, executor: SingleThreadedExecutor):
  try:
    executor.spin()
  except Exception as e:
    print(f"ROS spin error: {e}")


def main(args=None):
  rclpy.init(args=args)

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
    # await robot.play_note(Note.A5, 0.5)
    node.start_execution()
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
