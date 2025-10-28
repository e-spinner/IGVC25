#!/usr/bin/env python3
import rclpy
import math as m
from typing import Final
from rclpy.node import Node
from hack13.msg import Angle
from hack13.msg import AckState


Radius: Final[float] = 19.05  # [mm]


class AckCalc(Node):
  def __init__(self):
    super().__init__("ack_calc")

    self.Ideal_Angle_Sub = self.create_subscription(
      Angle, "/theta_ideal", self.Angle_Callback, 10
    )
    self.Ackermann_State_Pub = self.create_publisher(
      AckState, "/ack_state", 10
    )

    self.get_logger().info("ack_calc init()")

  def Angle_Callback(self, msg: Angle):
    Ideal_Angle = m.degrees(msg.theta)
    Ackermann_Angles = self.Linkage_Angles(Ideal_Angle, Radius)
    Output_Msg = AckState()
    Output_Msg.theta_2 = m.radians(Ackermann_Angles[0]) - 3.14
    Output_Msg.theta_3 = m.radians(Ackermann_Angles[1]) + 3.14
    Output_Msg.theta_4 = m.radians(Ackermann_Angles[2])
    Output_Msg.theta_5 = m.radians(Ackermann_Angles[3])
    Output_Msg.theta_pin = -m.radians(Ideal_Angle) * 1.652
    Output_Msg.d = -(Ackermann_Angles[4] - 131.064) / 1000.0
    self.Ackermann_State_Pub.publish(Output_Msg)

  def Crank_Slider(self, a: float, b: float, c: float, Theta_2: float):
    Theta_3 = m.degrees(m.asin((a * m.sin(m.radians(Theta_2)) - c) / b) + m.pi)
    d = a * m.cos(m.radians(Theta_2)) - b * m.cos(m.radians(Theta_3))
    return d

  def Slider_Crank_Theta2(self, K_1: float, K_2: float, K_3: float):
    A = K_1 - K_3
    B = 2 * K_2
    C = K_1 + K_3
    Theta_2_Plus = m.degrees(
      2 * m.atan((-B + m.sqrt(B**2 - 4 * A * C)) / (2 * A))
    )
    Theta_2_Minus = m.degrees(
      2 * m.atan((-B - m.sqrt(B**2 - 4 * A * C)) / (2 * A))
    )
    return [Theta_2_Plus, Theta_2_Minus]

  def Slider_Crank_Theta3(self, a: float, c: float, b: float, Theta2: list):
    Theta_3_Positive = (
      m.degrees(m.asin(-(a * m.sin(m.radians(Theta2[0])) - c) / (b))) + 180
    )
    Theta_3_Negative = (
      m.degrees(m.asin(-(a * m.sin(m.radians(Theta2[1])) - c) / (b))) + 180
    )
    return [Theta_3_Positive, Theta_3_Negative]

  def Pinion_Move(self, Ideal_Angle: float = 0, Radius=19.05):
    """treat as angle"""
    Move = m.radians(Ideal_Angle) * 1.652 * Radius
    return Move

  def Linkage_Angles(self, Ideal_Angle: float = 0, Pinion_Radius: float = 0):
    # Link Lengths
    a = 38.1  # [mm]
    b = 127  # [mm]
    c = -31.5  # [mm]
    d = (
      131.064 + self.Pinion_Move(Ideal_Angle, Pinion_Radius)
    )  # [mm] min = 100.85 mm max = 161.27 mm difference: 60.42 mm neutral: 131.064 MinMove: -30.21 MaxMove: 30.21

    # Left Side of the linkage
    K_1 = a**2 - b**2 + c**2 + d**2
    K_2 = -2 * a * c
    K_3 = -2 * a * d
    Theta_2 = self.Slider_Crank_Theta2(K_1, K_2, K_3)
    Theta_3 = self.Slider_Crank_Theta3(a, c, b, Theta_2)
    # print(Theta_2[1])
    # print(Theta_3[1])

    # Right Side of the linkage
    e = a
    f = b
    g = c
    h = (131.064 - d) + 131.064
    K_4 = e**2 - f**2 + g**2 + h**2
    K_5 = -2 * e * g
    K_6 = -2 * e * h
    Theta_4 = self.Slider_Crank_Theta2(K_4, K_5, K_6)
    Theta_5 = self.Slider_Crank_Theta3(e, g, f, Theta_4)
    Theta_4[1] = -(Theta_4[1] + 180)
    Theta_5[1] = 180 - Theta_5[1]

    Angles = [Theta_2[1], Theta_3[1], Theta_4[1], Theta_5[1], d]
    return Angles


def main(args=None):
  rclpy.init(args=None)

  node = AckCalc()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
