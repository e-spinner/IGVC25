#!/usr/bin/env python3
"""Differential-drive odometry from one hall sensor per drive wheel (GPIO rising edges)."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Optional

_AGENT_DEBUG_LOG_PATH = "/home/dev/IGVC25/.cursor/debug-369756.log"


def _agent_debug_log(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    # #region agent log
    try:
        rec = {
            "sessionId": "369756",
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }
        with open(_AGENT_DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
    except Exception:
        pass
    # #endregion

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


class HallOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("hall_odom")

        self.declare_parameter("left_hall_pin", 17)
        self.declare_parameter("right_hall_pin", 27)
        self.declare_parameter("gpio_mode_bcm", True)
        self.declare_parameter("pulses_per_revolution", 4.0)
        self.declare_parameter("wheel_radius_m", 0.1524)
        self.declare_parameter("track_width_m", 0.9144)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("pulse_timeout_s", 0.35)
        self.declare_parameter("gpio_bounce_ms", 3)
        self.declare_parameter("gpio_poll_period_s", 0.0005)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("odom_topic", "/odometry/wheel")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("simulate", False)

        self._ppr = max(float(self.get_parameter("pulses_per_revolution").value), 1e-6)
        self._r = float(self.get_parameter("wheel_radius_m").value)
        self._track = max(float(self.get_parameter("track_width_m").value), 1e-6)
        self._timeout = float(self.get_parameter("pulse_timeout_s").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._child_frame = str(self.get_parameter("child_frame_id").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._simulate = _as_bool(self.get_parameter("simulate").value)

        self._lock = threading.Lock()
        self._t_left: Optional[float] = None
        self._t_right: Optional[float] = None
        self._dt_left: Optional[float] = None
        self._dt_right: Optional[float] = None

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_int_time = time.monotonic()

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
        self._sim_t0 = time.monotonic()

        if self._simulate:
            self.get_logger().info("Hall odometry in simulate mode (no GPIO).")
        else:
            self._setup_gpio()

    def _setup_gpio(self) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError:
            self.get_logger().warn(
                "RPi.GPIO not available; use simulate:=true on non-Pi or pip install."
            )
            self._simulate = True
            return

        self._gpio = GPIO
        left_pin = int(self.get_parameter("left_hall_pin").value)
        right_pin = int(self.get_parameter("right_hall_pin").value)
        bounce = int(self.get_parameter("gpio_bounce_ms").value)

        if bool(self.get_parameter("gpio_mode_bcm").value):
            GPIO.setmode(GPIO.BCM)
        else:
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        model = ""
        try:
            with open("/proc/device-tree/model", "rb") as mf:
                model = mf.read().decode(errors="ignore").strip("\x00").strip()
        except OSError:
            pass
        sysfs_export_w = os.access("/sys/class/gpio/export", os.W_OK)
        poll_period = max(float(self.get_parameter("gpio_poll_period_s").value), 1e-5)
        # #region agent log
        _agent_debug_log(
            "H1",
            "hall_odom_node.py:_setup_gpio",
            "pre edge-detect context",
            {
                "left_pin": left_pin,
                "right_pin": right_pin,
                "bounce_ms": bounce,
                "sysfs_export_writable": sysfs_export_w,
                "model": model,
                "poll_period_s": poll_period,
            },
        )
        # #endregion

        def left_cb(_ch: int) -> None:
            self._edge("left")

        def right_cb(_ch: int) -> None:
            self._edge("right")

        edge_err: Optional[RuntimeError] = None
        try:
            GPIO.add_event_detect(left_pin, GPIO.RISING, callback=left_cb, bouncetime=bounce)
            GPIO.add_event_detect(right_pin, GPIO.RISING, callback=right_cb, bouncetime=bounce)
        except RuntimeError as e:
            edge_err = e
            for p in (left_pin, right_pin):
                try:
                    GPIO.remove_event_detect(p)
                except Exception:
                    pass

        if edge_err is not None:
            # #region agent log
            _agent_debug_log(
                "H1",
                "hall_odom_node.py:_setup_gpio",
                "add_event_detect failed; falling back to polling",
                {"err": str(edge_err), "sysfs_export_writable": sysfs_export_w},
            )
            # #endregion
            self.get_logger().warn(
                f"GPIO edge detection unavailable ({edge_err}); using polled sampling instead."
            )
            self._poll_stop = threading.Event()
            self._poll_thread = threading.Thread(
                target=self._gpio_poll_loop,
                args=(left_pin, right_pin, bounce, poll_period),
                daemon=True,
                name="hall_gpio_poll",
            )
            self._poll_thread.start()
            # #region agent log
            _agent_debug_log(
                "H2",
                "hall_odom_node.py:_setup_gpio",
                "polling thread started",
                {"left_pin": left_pin, "right_pin": right_pin},
            )
            # #endregion
            self.get_logger().info(
                f"Hall GPIO polling BCM pins left={left_pin} right={right_pin} "
                f"bounce={bounce}ms period={poll_period * 1e6:.0f}us"
            )
        else:
            self.get_logger().info(
                f"Hall GPIO BCM pins left={left_pin} right={right_pin} bounce={bounce}ms (interrupts)"
            )

    def _gpio_poll_loop(
        self, left_pin: int, right_pin: int, bounce_ms: int, period_s: float
    ) -> None:
        GPIO = self._gpio
        stop_ev = self._poll_stop
        if GPIO is None or stop_ev is None:
            return
        debounce_s = max(int(bounce_ms), 1) / 1000.0
        prev_l = int(GPIO.input(left_pin))
        prev_r = int(GPIO.input(right_pin))
        last_l = 0.0
        last_r = 0.0
        while not stop_ev.wait(period_s):
            now = time.monotonic()
            l = int(GPIO.input(left_pin))
            r = int(GPIO.input(right_pin))
            if prev_l == 0 and l == 1 and (now - last_l) >= debounce_s:
                self._edge("left")
                last_l = now
            if prev_r == 0 and r == 1 and (now - last_r) >= debounce_s:
                self._edge("right")
                last_r = now
            prev_l, prev_r = l, r

    def _edge(self, side: str) -> None:
        now = time.monotonic()
        with self._lock:
            if side == "left":
                if self._t_left is not None:
                    dt = now - self._t_left
                    if 1e-4 < dt < 3.0:
                        self._dt_left = dt
                self._t_left = now
            else:
                if self._t_right is not None:
                    dt = now - self._t_right
                    if 1e-4 < dt < 3.0:
                        self._dt_right = dt
                self._t_right = now

    def _wheel_speeds(self, now: float) -> tuple[float, float]:
        """Linear speed of each wheel contact (m/s), left/right."""
        if self._simulate:
            # Slow figure-eight-ish motion for bench testing TF tree.
            t = now - self._sim_t0
            v_l = 0.15 * math.sin(0.3 * t)
            v_r = 0.15 * math.cos(0.3 * t)
            return v_l, v_r

        dtheta = 2.0 * math.pi / self._ppr
        v_left = 0.0
        v_right = 0.0

        with self._lock:
            if self._t_left is not None and (now - self._t_left) < self._timeout:
                if self._dt_left is not None and self._dt_left > 1e-6:
                    v_left = self._r * dtheta / self._dt_left
            if self._t_right is not None and (now - self._t_right) < self._timeout:
                if self._dt_right is not None and self._dt_right > 1e-6:
                    v_right = self._r * dtheta / self._dt_right

        return v_left, v_right

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_int_time
        self._last_int_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / 50.0

        v_l, v_r = self._wheel_speeds(now)
        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / self._track

        self._yaw += omega * dt
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self._yaw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Covariances: trust twist more than integrated pose for EKF tuning.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self._pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._frame_id
            t.child_frame_id = self._child_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quat(self._yaw)
            self._tf_broadcaster.sendTransform(t)

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
