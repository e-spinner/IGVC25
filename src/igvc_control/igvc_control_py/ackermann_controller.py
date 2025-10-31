#!/usr/bin/env python3
import rclpy
import math as m
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from rclpy.node import Node
from igvc.msg import Angle
from igvc.msg import AckState


class AckermannController(Node):
  def __init__(self):
    super().__init__("ackermann_controller")

    # Declare and get linkage parameters in meters
    self.declare_parameter("pinion_radius", 0.01905)
    self.declare_parameter("steering_arm_length", 0.0381)
    self.declare_parameter("tie_rod_length", 0.127)
    self.declare_parameter("rack_offset_x", -0.0315)
    self.declare_parameter("rack_neutral_y", 0.131064)
    self.declare_parameter("pinion_gear_ratio", 1.652)
    self.declare_parameter("max_pinion_angle", 2.0)
    self.declare_parameter("wheel_angle", 0.32253)

    # Vehicle geometry parameters
    self.declare_parameter("wheelbase", 0.42)
    self.declare_parameter("track_width", 0.36)
    self.declare_parameter("linkage_num", 1)

    # Get parameters in meters
    self.pinion_radius = self.get_parameter("pinion_radius").value  # [m]
    self.link_a = self.get_parameter("steering_arm_length").value  # [m]
    self.link_b = self.get_parameter("tie_rod_length").value  # [m]
    self.link_c = self.get_parameter("rack_offset_x").value  # [m]
    self.rack_neutral = self.get_parameter("rack_neutral_y").value  # [m]
    self.pinion_gear_ratio = self.get_parameter("pinion_gear_ratio").value
    self.max_pinion_angle = self.get_parameter("max_pinion_angle").value  # [rad]
    self.wheel_angle = self.get_parameter("wheel_angle").value  # [rad]
    self.wheelbase = self.get_parameter("wheelbase").value  # [m]
    self.track_width = self.get_parameter("track_width").value  # [m]
    self.linkage_num = self.get_parameter("linkage_num").value  # [m]

    self.Ideal_Angle_Sub = self.create_subscription(
      Angle, "/theta_ideal", self.angle_callback, 10
    )
    self.Ackermann_State_Pub = self.create_publisher(AckState, "/ack_state", 10)

    self.calibration_func = self.build_calibration()

    self.get_logger().info(
      f"ack_calc init() with pinion_radius={self.pinion_radius:.5f}m, "
      f"link_a={self.link_a:.5f}m, link_b={self.link_b:.5f}m, "
      f"wheelbase={self.wheelbase:.5f}m, track_width={self.track_width:.5f}m"
    )

  def angle_callback(self, msg: Angle):
    ideal_angle = msg.theta  # Already in radians

    pinion_angle = self.calibration_func(ideal_angle)

    ackermann_angles = self.Linkage_Angles(pinion_angle)  # type: ignore
    Output_Msg = AckState()
    Output_Msg.theta_2 = ackermann_angles[0] - m.pi
    Output_Msg.theta_3 = ackermann_angles[1] + m.pi
    Output_Msg.theta_4 = ackermann_angles[2]
    Output_Msg.theta_5 = ackermann_angles[3]
    Output_Msg.theta_pin = -pinion_angle * self.pinion_gear_ratio  # type: ignore
    Output_Msg.d = -(ackermann_angles[4] - self.rack_neutral)
    self.Ackermann_State_Pub.publish(Output_Msg)

  def Linkage_Angles(self, pinion_angle: float = 0):
    # Use parameterized link lengths
    a = self.link_a  # [m] steering arm length
    b = self.link_b  # [m] tie rod length
    c = self.link_c  # [m] rack offset
    d = self.rack_neutral + (
      pinion_angle * self.pinion_gear_ratio * self.pinion_radius  # type: ignore
    )  # [m] rack position

    assert a is not None
    assert b is not None
    assert c is not None

    def Slider_Crank_Theta2(K_1: float, K_2: float, K_3: float):
      A = K_1 - K_3
      B = 2 * K_2
      C = K_1 + K_3
      theta_2_plus = 2 * m.atan((-B + m.sqrt(B**2 - 4 * A * C)) / (2 * A))
      theta_2_minus = 2 * m.atan((-B - m.sqrt(B**2 - 4 * A * C)) / (2 * A))
      return [theta_2_plus, theta_2_minus]

    def Slider_Crank_Theta3(a: float, c: float, b: float, theta2: list):
      theta_3_positive = m.asin(-(a * m.sin(theta2[0]) - c) / b) + m.pi
      theta_3_negative = m.asin(-(a * m.sin(theta2[1]) - c) / b) + m.pi
      return [theta_3_positive, theta_3_negative]

    # Left Side of the linkage
    K_1 = a**2 - b**2 + c**2 + d**2
    K_2 = -2 * a * c
    K_3 = -2 * a * d
    theta_2 = Slider_Crank_Theta2(K_1, K_2, K_3)
    theta_3 = Slider_Crank_Theta3(a, c, b, theta_2)

    # Right Side of the linkage
    e = a
    f = b
    g = c
    h = (self.rack_neutral - d) + self.rack_neutral
    K_4 = e**2 - f**2 + g**2 + h**2
    K_5 = -2 * e * g
    K_6 = -2 * e * h
    theta_4 = Slider_Crank_Theta2(K_4, K_5, K_6)
    theta_5 = Slider_Crank_Theta3(e, g, f, theta_4)
    theta_4[1] = -(theta_4[1] + m.pi)
    theta_5[1] = m.pi - theta_5[1]

    angles = [theta_2[1], theta_3[1], theta_4[1], theta_5[1], d]
    return angles

  def build_calibration(self):
    assert self.max_pinion_angle is not None

    pinion_angles = np.linspace(-self.max_pinion_angle, self.max_pinion_angle, 1024)

    def r_actual(θ_l: float, θ_r: float) -> float:
      return (self.wheelbase) / (2 * (m.tan(θ_l) + m.tan(θ_r)))  # type: ignore

    def θ_ideal(radius: float) -> float:
      return m.atan(self.wheelbase / radius)  # type: ignore

    left_wheel_angles = []
    right_wheel_angles = []
    actual_radii = []
    ideal_radii = []
    ideal_angles = []
    for pinion_angle in pinion_angles:
      angles = self.Linkage_Angles(pinion_angle)  # type: ignore

      θ_left = angles[0] + m.pi + self.wheel_angle
      θ_right = angles[2] - self.wheel_angle
      left_wheel_angles.append(θ_left)
      right_wheel_angles.append(θ_right)

      r = r_actual(θ_left, θ_right)
      actual_radii.append(r)
      i = θ_ideal(r)
      ideal_angles.append(i)
      r_ideal = self.wheelbase / m.tan(i) if i != 0 else float("inf")  # type: ignore
      ideal_radii.append(r_ideal)

    # graph wheel angles
    fig, ax = plt.subplots()
    ax.plot(ideal_angles, left_wheel_angles, color="blue", label="left wheel angle")
    ax.plot(ideal_angles, right_wheel_angles, color="red", label="rigth wheel angle")
    ax.plot(ideal_angles, pinion_angles, color="green", label="pinion angle")
    ax.legend()
    ax.set(
      xlabel="ideal steering angle [rad]",
      ylabel="wheel angle [rad]",
      title="steering angle vs. wheel angles",
    )
    fig.savefig(f"wheel_angles: {self.linkage_num}", dpi=600)  # type: ignore

    # Graph turning radius comparison
    fig2, ax2 = plt.subplots()
    ax2.plot(ideal_angles, actual_radii, color="blue", label="actual turning radius")
    ax2.plot(
      ideal_angles,
      ideal_radii,
      color="red",
      linestyle="--",
      label="ideal turning radius",
    )
    ax2.legend()
    ax2.set(
      xlabel="ideal steering angle [rad]",
      ylabel="turning radius [m]",
      title="actual vs. ideal turning radius",
    )
    ax2.grid(True, alpha=0.3)
    fig2.savefig(f"turning_radius: {self.linkage_num}", dpi=600)  # type: ignore

    caligration_func = interp1d(
      pinion_angles,
      ideal_angles,
      kind="linear",
      bounds_error=False,
    )

    return caligration_func


def main(args=None):
  rclpy.init(args=args)

  node = AckermannController()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
