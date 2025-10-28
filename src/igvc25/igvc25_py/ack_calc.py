#!/usr/bin/env python3
import rclpy
import math as m
from rclpy.node import Node
from igvc25.msg import Angle
from igvc25.msg import AckState


class AckCalc(Node):
  def __init__(self):
    super().__init__("ack_calc")

    # Declare and get linkage parameters (convert from meters to millimeters)
    self.declare_parameter("pinion_radius", 0.01905)
    self.declare_parameter("steering_arm_length", 0.0381)
    self.declare_parameter("tie_rod_length", 0.127)
    self.declare_parameter("rack_offset_x", -0.0315)
    self.declare_parameter("rack_neutral_y", 0.131064)
    self.declare_parameter("pinion_gear_ratio", 1.652)

    # Get parameters and convert to mm for calculations
    self.pinion_radius = (
      self.get_parameter("pinion_radius").value * 1000.0
    )  # [mm]
    self.link_a = (
      self.get_parameter("steering_arm_length").value * 1000.0
    )  # [mm]
    self.link_b = self.get_parameter("tie_rod_length").value * 1000.0  # [mm]
    self.link_c = self.get_parameter("rack_offset_x").value * 1000.0  # [mm]
    self.rack_neutral = (
      self.get_parameter("rack_neutral_y").value * 1000.0
    )  # [mm]
    self.pinion_gear_ratio = self.get_parameter("pinion_gear_ratio").value

    self.Ideal_Angle_Sub = self.create_subscription(
      Angle, "/theta_ideal", self.Angle_Callback, 10
    )
    self.Ackermann_State_Pub = self.create_publisher(
      AckState, "/ack_state", 10
    )

    self.get_logger().info(
      f"ack_calc init() with pinion_radius={self.pinion_radius:.3f}mm, "
      f"link_a={self.link_a:.3f}mm, link_b={self.link_b:.3f}mm"
    )

  def Angle_Callback(self, msg: Angle):
    Ideal_Angle = m.degrees(msg.theta)
    Ackermann_Angles = self.Linkage_Angles(Ideal_Angle, self.pinion_radius)
    Output_Msg = AckState()
    Output_Msg.theta_2 = m.radians(Ackermann_Angles[0]) - 3.14
    Output_Msg.theta_3 = m.radians(Ackermann_Angles[1]) + 3.14
    Output_Msg.theta_4 = m.radians(Ackermann_Angles[2])
    Output_Msg.theta_5 = m.radians(Ackermann_Angles[3])
    Output_Msg.theta_pin = -m.radians(Ideal_Angle) * self.pinion_gear_ratio
    Output_Msg.d = -(Ackermann_Angles[4] - self.rack_neutral) / 1000.0
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

  def Pinion_Move(self, Ideal_Angle: float = 0, Radius: float = 19.05):
    """treat as angle"""
    Move = m.radians(Ideal_Angle) * self.pinion_gear_ratio * Radius
    return Move

  def Linkage_Angles(self, Ideal_Angle: float = 0, Pinion_Radius: float = 0):
    # Use parameterized link lengths
    a = self.link_a  # [mm] steering arm length
    b = self.link_b  # [mm] tie rod length
    c = self.link_c  # [mm] rack offset
    d = self.rack_neutral + self.Pinion_Move(
      Ideal_Angle, Pinion_Radius
    )  # [mm] rack position

    # Left Side of the linkage
    K_1 = a**2 - b**2 + c**2 + d**2
    K_2 = -2 * a * c
    K_3 = -2 * a * d
    Theta_2 = self.Slider_Crank_Theta2(K_1, K_2, K_3)
    Theta_3 = self.Slider_Crank_Theta3(a, c, b, Theta_2)

    # Right Side of the linkage
    e = a
    f = b
    g = c
    h = (self.rack_neutral - d) + self.rack_neutral
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
