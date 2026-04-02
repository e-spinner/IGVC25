# Todo

made in depth via gemini, organized by goal

## Manual Control

1. Steering safety and homing hardening in firmware (`ackermann_firmware.ino`)
   - Add explicit steer fault state (`homing timeout`, `both limits active`, `invalid travel`).
   - Add limit-switch debounce (hardware and/or software).
   - Add watchdog behavior for stale serial commands (stop drive + steering).
   - Confirm limit-switch polarity and direction conventions on physical hardware.

2. Steering state and command mapping validation
   - Verify encoder-to-pinion conversion after homing across full left/right sweep.
   - Confirm `g_pinion_cmd` sign maps to expected physical direction.
   - Tune `ENCODER_COUNTS_PER_RAD`, PID gains, and `PINION_CMD_TOLERANCE`.
   - Revisit placeholder assumptions in steering control path and document final model.

3. Velocity path consistency and cap enforcement
   - Verify units end-to-end: `Twist.linear.x (m/s)` -> controller conversion -> firmware motor command.
   - Keep 5 mph cap (`2.2352 m/s`) consistent in joystick, controller, and firmware.
   - Validate reverse behavior and zero-command behavior under normal and fault conditions.

4. Launch and operator workflow robustness
   - Validate `manual-ctrl.launch.py` on Jetson with real Xbox controller (Bluetooth + USB fallback).
   - Confirm `use_sim_time` behavior is consistent across included launch files.
   - Ensure required ROS packages and udev/device permissions are documented and reproducible.

5. Serial and hardware interface reliability
   - Validate baud/timeout settings across firmware, URDF/ros2_control, and launch args.
   - Add clearer diagnostics in hardware interface for parse failures and serial reconnect scenarios.
   - Confirm startup/shutdown transitions do not leave motors active.

## Autonomous Control

1. Ackermann command semantics in controller layer
   - Decide whether joystick/nav `angular.z` represents ideal steering angle or pinion angle.
   - Re-enable and validate calibration mapping path in `ackermann_angle_controller` if needed.
   - Align controller clamp limits with linkage/mechanical constraints from config.

2. Pure pursuit velocity model and tuning
   - Parameterize centripetal acceleration limit (instead of compile-time constant).
   - Determine realistic `a_max` from tire/surface tests and steering actuator response.
   - Validate curvature-based speed profile against stopping distance and path tracking error.

3. Navigation stack integration and command arbitration
   - Verify nav command topic chain through `twist_mux` and priority rules.
   - Confirm manual override behavior is deterministic and safe.
   - Add explicit estop topic/path into mux with highest priority.

4. Localization and state estimation checks
   - Validate EKF/navsat configuration with real sensor timing and frame conventions.
   - Confirm transform tree stability under motion and during GNSS dropout.
   - Add tests/log review for covariance behavior and failure modes.

5. Scenario-based autonomous testing
   - Build repeatable test cases: straight, slalom, tight turns, recovery after obstacle stop.
   - Record and review rosbags for command/state/TF timing and actuator saturation.
   - Define pass/fail metrics for path tracking, overshoot, and intervention frequency.

## Competition Ready

1. Safety architecture
   - Implement and validate layered estop (wireless/hardware/software).
   - Define fail-safe defaults on node crash, serial loss, sensor loss, or controller timeout.
   - Add startup interlocks (no drive until homing + health checks pass).

2. Configuration and release discipline
   - Freeze and version control all competition-critical constants and profiles.
   - Separate sim/dev/comp configs cleanly and document selection process.
   - Add configuration sanity checks at launch/runtime (invalid limits, missing files, bad params).

3. Observability and debugging
   - Standardize logs/status topics for health, faults, mode, and actuator state.
   - Add concise operator-facing status output for pit-side diagnostics.
   - Create a minimal post-run analysis checklist using bag playback.

4. Validation coverage and regression prevention
   - Add automated checks for build/lint/static analysis in CI.
   - Add unit/integration tests for controller math and parsing edge cases.
   - Maintain a regression suite for previously found failures (limit switch, timeout, sign errors).

5. Field operations readiness
   - Define pre-run checklist: hardware inspect, calibration/homing verify, topic health verify.
   - Define runbook for common faults and recovery actions.
   - Perform endurance tests and repeated power-cycle tests to catch intermittent failures.