#!/usr/bin/env python3
"""
Differential-drive odometry from two hall sensors per wheel (four GPIO pins).

Mounting (why this is not “two sensors in the same place”):
  The two halls sit near each other but are offset along the direction the rim
  moves, with a gap that is large compared to each magnet. As a magnet passes,
  one sensor toggles before the other, so forward rotation produces a rising
  edge train like A,B,A,B,... and reverse produces B,A,B,A,... That order is
  the same idea as quadrature: at the instant A rises, B is already in a state
  that depends on which sensor saw the field first, so we get direction without
  tracking a long string—one read of B at each A rising edge is enough.

Timing (~66 Hz worst case on one channel):
  About 15 ms between successive rises on the same pin. Polling at
  gpio_poll_period_s (default 0.5 ms) gives many samples per edge; keep
  gpio_bounce_ms well below that spacing (a few ms) so debounce does not swallow
  real edges.

Each wheel counts rising edges on A (three per revolution with three magnets).
Velocity uses a rolling window over signed pulses in velocity_window_s.

GPIO is polled in a background thread (RPi.GPIO reads only).
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RollingPulseWindow:
    """
    Stores (timestamp, signed_pulse) for one wheel. A signed pulse is +1 or -1.

    Speed is (sum of signed pulses) * meters_per_pulse divided by the time
    span from the oldest kept event to "now". If we have fewer than two
    timestamps inside the window, speed is reported as 0.
    """

    def __init__(self, window_s: float) -> None:
        self._window_s = max(float(window_s), 1e-3)
        self._events: Deque[Tuple[float, int]] = deque()

    def add_pulse(self, t: float, direction_sign: int) -> None:
        """Record one pulse at time t; direction_sign should be +1 or -1."""
        self._events.append((t, 1 if direction_sign > 0 else -1))
        self._trim(t)

    def _trim(self, t: float) -> None:
        cutoff = t - self._window_s
        while self._events and self._events[0][0] < cutoff:
            self._events.popleft()

    def linear_speed_m_s(self, t: float, meters_per_pulse: float) -> float:
        """Net wheel rim speed (m/s) implied by pulses inside the window."""
        self._trim(t)
        if len(self._events) < 2:
            return 0.0
        t0 = self._events[0][0]
        dt = t - t0
        if dt < 1e-6:
            return 0.0
        total_pulses = sum(s for _, s in self._events)
        return (total_pulses * meters_per_pulse) / dt


class HallOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("hall_odom")

        # --- GPIO (BCM): two pins per side; A = count, B = direction at A rise (see module doc) ---
        self.declare_parameter("left_a_pin", 17)
        self.declare_parameter("left_b_pin", 18)
        self.declare_parameter("right_a_pin", 22)
        self.declare_parameter("right_b_pin", 23)
        self.declare_parameter("gpio_mode_bcm", True)
        # Min time between accepted rises on the *same* pin (same wheel). Keep
        # this smaller than the fastest same-pin edge spacing (~1/66 s ≈ 15 ms).
        self.declare_parameter("gpio_bounce_ms", 3)
        self.declare_parameter("gpio_poll_period_s", 0.0005)

        # --- Wheel geometry ---
        # Three rising edges on channel A per full wheel turn (three magnets).
        self.declare_parameter("pulses_per_revolution", 3.0)
        # 4 inch diameter => radius 2 inches = 0.0508 m
        self.declare_parameter("wheel_radius_m", 0.0508)
        self.declare_parameter("track_width_m", 0.9144)

        # How far back we look to estimate speed from pulse counts.
        self.declare_parameter("velocity_window_s", 0.25)

        # True: forward = A rose while B reads high (stagger geometry / wiring).
        # False: forward when B is low at that instant. Swap if the robot runs
        # backward, or use invert_left / invert_right.
        self.declare_parameter("forward_when_b_high", True)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        # --- ROS I/O ---
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("odom_topic", "/odometry/wheel")
        self.declare_parameter("publish_tf", False)

        ppr = max(float(self.get_parameter("pulses_per_revolution").value), 1e-6)
        self._m_per_pulse = (2.0 * math.pi * float(self.get_parameter("wheel_radius_m").value)) / ppr
        self._track = max(float(self.get_parameter("track_width_m").value), 1e-6)
        self._vel_window_s = float(self.get_parameter("velocity_window_s").value)

        self._forward_b_high = _as_bool(self.get_parameter("forward_when_b_high").value)
        self._invert_left = _as_bool(self.get_parameter("invert_left").value)
        self._invert_right = _as_bool(self.get_parameter("invert_right").value)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._child_frame = str(self.get_parameter("child_frame_id").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._lock = threading.Lock()
        self._left_win = RollingPulseWindow(self._vel_window_s)
        self._right_win = RollingPulseWindow(self._vel_window_s)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self._gpio = None
        self._poll_stop: Optional[threading.Event] = None
        self._poll_thread: Optional[threading.Thread] = None

        self._setup_gpio_poll_thread()

    def _setup_gpio_poll_thread(self) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError as e:
            self.get_logger().fatal(
                "RPi.GPIO is required on the Raspberry Pi. Install it (e.g. pip install RPi.GPIO)."
            )
            raise RuntimeError("RPi.GPIO not available") from e

        self._gpio = GPIO
        la = int(self.get_parameter("left_a_pin").value)
        lb = int(self.get_parameter("left_b_pin").value)
        ra = int(self.get_parameter("right_a_pin").value)
        rb = int(self.get_parameter("right_b_pin").value)
        bounce_ms = max(int(self.get_parameter("gpio_bounce_ms").value), 0)
        period_s = max(float(self.get_parameter("gpio_poll_period_s").value), 1e-5)

        if bool(self.get_parameter("gpio_mode_bcm").value):
            GPIO.setmode(GPIO.BCM)
        else:
            GPIO.setmode(GPIO.BOARD)

        for p in (la, lb, ra, rb):
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._poll_stop = threading.Event()
        self._poll_thread = threading.Thread(
            target=self._gpio_poll_loop,
            args=(la, lb, ra, rb, bounce_ms, period_s),
            daemon=True,
            name="hall_gpio_poll",
        )
        self._poll_thread.start()
        self.get_logger().info(
            f"Hall GPIO polling: left A={la} B={lb}, right A={ra} B={rb}, "
            f"bounce={bounce_ms} ms, period={period_s * 1e6:.0f} us"
        )

    def _direction_from_b(self, b_level: int) -> int:
        """
        +1 / -1 from B at the sample where A just went high.

        With sensors staggered along the rim, “A first” vs “B first” on each
        magnet makes B stable enough here to encode the same information as an
        ABAB... vs BABA... edge order, without a separate sequence buffer.
        """
        forward = b_level == 1 if self._forward_b_high else b_level == 0
        return 1 if forward else -1

    def _apply_left_pulse(self, t: float, b_level: int) -> None:
        sign = self._direction_from_b(b_level)
        if self._invert_left:
            sign = -sign
        ds_l = sign * self._m_per_pulse
        ds_r = 0.0
        self._integrate_pose(ds_l, ds_r)
        self._left_win.add_pulse(t, sign)

    def _apply_right_pulse(self, t: float, b_level: int) -> None:
        sign = self._direction_from_b(b_level)
        if self._invert_right:
            sign = -sign
        ds_l = 0.0
        ds_r = sign * self._m_per_pulse
        self._integrate_pose(ds_l, ds_r)
        self._right_win.add_pulse(t, sign)

    def _integrate_pose(self, ds_l: float, ds_r: float) -> None:
        """
        One differential-drive step from small left/right arc lengths (meters).

        Only one wheel should move per call from our hall logic, but the math
        works for any small pair (ds_l, ds_r).
        """
        ds = 0.5 * (ds_l + ds_r)
        d_yaw = (ds_r - ds_l) / self._track
        yaw_mid = self._yaw + 0.5 * d_yaw
        self._x += ds * math.cos(yaw_mid)
        self._y += ds * math.sin(yaw_mid)
        self._yaw += d_yaw

    def _gpio_poll_loop(
        self,
        left_a: int,
        left_b: int,
        right_a: int,
        right_b: int,
        bounce_ms: int,
        period_s: float,
    ) -> None:
        GPIO = self._gpio
        stop_ev = self._poll_stop
        if GPIO is None or stop_ev is None:
            return

        debounce_s = max(bounce_ms, 1) / 1000.0
        prev_la = int(GPIO.input(left_a))
        prev_ra = int(GPIO.input(right_a))
        last_l = 0.0
        last_r = 0.0

        while not stop_ev.wait(period_s):
            now = time.monotonic()
            la = int(GPIO.input(left_a))
            lb = int(GPIO.input(left_b))
            ra = int(GPIO.input(right_a))
            rb = int(GPIO.input(right_b))

            # Rising edge on A: idle/low -> high, with debounce per wheel.
            if prev_la == 0 and la == 1 and (now - last_l) >= debounce_s:
                with self._lock:
                    self._apply_left_pulse(now, lb)
                last_l = now

            if prev_ra == 0 and ra == 1 and (now - last_r) >= debounce_s:
                with self._lock:
                    self._apply_right_pulse(now, rb)
                last_r = now

            prev_la, prev_ra = la, ra

    def _on_timer(self) -> None:
        now = time.monotonic()

        with self._lock:
            v_l = self._left_win.linear_speed_m_s(now, self._m_per_pulse)
            v_r = self._right_win.linear_speed_m_s(now, self._m_per_pulse)
        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / self._track

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame

        with self._lock:
            odom.pose.pose.position.x = self._x
            odom.pose.pose.position.y = self._y
            odom.pose.pose.orientation = yaw_to_quat(self._yaw)

        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self._pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._frame_id
            tf_msg.child_frame_id = self._child_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self) -> bool:
        if self._poll_stop is not None:
            self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None
        self._poll_stop = None
        if self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HallOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
