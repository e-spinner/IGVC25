#!/usr/bin/env python3
import os
import json
import asyncio
from threading import Thread

import rclpy
from rclpy.node import Node

from irobot_edu_sdk.backend.usb import USB
from irobot_edu_sdk.robots import Root, Robot, event, hand_over, Color, Create3
from irobot_edu_sdk.music import Note

from igvc25.msg import RootInstruction


class RootController(Node):
  def __init__(self):
    super().__init__("root_controller")

    self.declare_parameter("pattern", "line")
    self.declare_parameter("speed", 10.0)
    self.declare_parameter("cycles", 1)
    self.declare_parameter("distance", 1.0)
    self.declare_parameter("publish_rate", 10.0)

    # Get parameters
    self.pattern = self.get_parameter("pattern").value
    self.speed = self.get_parameter("speed").value
    self.cycles = self.get_parameter("cycles").value
    self.distance = self.get_parameter("distance").value
    self.publish_rate = self.get_parameter("publish_rate").value

    self.publisher = self.create_publisher(RootInstruction, "/root_instr", 10)

    # Generate instruction sequence
    self.instruction_sequence = self.generate_pattern_sequence()

    # Root setup
    self.robot = None
    self.backend = USB()
    self.event_loop = None
    self.robot_thread = None

    # State tracking
    self.current_instruction_idx = 0
    self.current_loop = 0
    self.is_executing = False
    self.sequence_complete = False

    self.get_logger().info(
      f"Root Controller initialized with pattern: {self.pattern}, "
      f"Speed: {self.speed} cm/s, Cycles: {self.cycles}, Distance: {self.distance} m"  # noqa: E501
    )

    # start robot connection in new thread
    self.start_robot_connection()

    self.timer = self.create_timer(1, self.timer_callback)
    self.count = 0

  def generate_pattern_sequence(self):
    pattern_generators = {"line": self.generate_line_pattern}

    if self.pattern not in pattern_generators:
      self.get_logger().error(
        f"Unkown pattern: {self.pattern}. Available: {list(pattern_generators.keys())}"  # noqa: E501
      )
      return []

    return pattern_generators[self.pattern]()

  def generate_line_pattern(self):
    # speed in cm/s, distance in m
    duration = (self.distance * 100) / abs(self.speed)  # type: ignore

    instructions = []
    for _ in range(self.cycles):  # type: ignore
      instructions.append(
        {"left": self.speed, "right": self.speed, "duration": duration}
      )

    return instructions

  def start_robot_connection(self):
    self.robot_thread = Thread(target=self.run_robot_async, daemon=True)
    self.robot_thread.start()

  def run_robot_async(self):
    self.event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self.event_loop)
    self.event_loop.run_until_complete(self.connect_robot())

  async def connect_robot(self):
    try:
      self.robot = Root(self.backend)
      self.get_logger().info("Attempting to connect to Root robot...")
      await self.robot.connect()  # type: ignore
      self.get_logger().info("Successfully connected to Root robot!")

      await self.robot.play_note(Note.A5, 0.2)

      while rclpy.ok():
        await asyncio.sleep(0.1)

    except Exception as e:
      self.get_logger().error(f"Failed to connect to Root robot: {e}")

  def stop_robot(self):
    if self.robot and self.event_loop:
      asyncio.run_coroutine_threadsafe(
        self.robot.set_wheel_speeds(0, 0), self.event_loop
      )
      self.get_logger().info("Robot stopped")

  def timer_callback(self):
    if self.sequence_complete:
      return

    if not self.instruction_sequence:
      self.get_logger().error("No instruction Sequence Generated!!!")
      self.sequence_complete = True
      return

    if not self.is_executing:
      if self.current_instruction_idx >= len(self.instruction_sequence):
        if not self.sequence_complete:
          self.get_logger().info("Pattern Completed!")
          self.sequence_complete = True
          self.stop_robot()
        return

      instr = self.instruction_sequence[self.current_instruction_idx]

      # publish intruction
      msg = RootInstruction()
      msg.left_speed = instr["left"]
      msg.right_speed = instr["right"]
      msg.duration = instr["duration"]

      self.publisher.publish(msg)

      self.get_logger().info(
        f"Instruction [{self.current_instruction_idx + 1}/{len(self.instruction_sequence)}]: "  # noqa: E501
      )

      # Execute on root
      self.execute_instruction(instr)

      self.current_instruction_idx += 1

  def execute_instruction(self, instr):
    if self.robot is None or self.event_loop is None:
      self.get_logger().error("Robot not connected, skipping execution!!!")
      return

    self.is_executing = True

    future = asyncio.run_coroutine_threadsafe(
      self.execute_instruction_async(instr), self.event_loop
    )

    future.add_done_callback(lambda f: setattr(self, "is_executing", False))

  async def execute_instruction_async(self, instr):
    try:
      left = int(instr["left"])
      right = int(instr["right"])
      duration = instr["duration"]

      await self.robot.set_wheel_speeds(left, right)  # pyright: ignore[reportOptionalMemberAccess]

      await asyncio.sleep(duration)

      await self.robot.set_wheel_speeds(0, 0)  # pyright: ignore[reportOptionalMemberAccess]

      await asyncio.sleep(0.1)

    except Exception as e:
      self.get_logger().error(f"Error execution instruction: {e}")


def main(args=None):
  rclpy.init(args=args)
  node = RootController()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
