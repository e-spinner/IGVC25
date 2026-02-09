#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace igvc_control {

class AckermannAngleController final
    : public controller_interface::ControllerInterface {
public:
  AckermannAngleController() : controller_interface::ControllerInterface() {}

  // MARK: INIT
  controller_interface::CallbackReturn on_init() override {
    try {
      m_node    = get_node();
      auto node = m_node.lock();
      m_logger  = node->get_logger();

      // Declare parameters
      // -----------------------------------------------------------------------------

      // Linkage Parameters
      node->declare_parameter("pinion_radius", 0.01905);
      node->declare_parameter("steering_arm_length", 0.0381);
      node->declare_parameter("tie_rod_length", 0.127);
      node->declare_parameter("rack_offset_x", -0.0315);
      node->declare_parameter("rack_neutral_y", 0.131064);
      node->declare_parameter("pinion_gear_ratio", 1.652);
      node->declare_parameter("max_pinion_angle", 2.0);
      node->declare_parameter("wheel_angle", 0.32253);
      node->declare_parameter("wheelbase", 0.42);
      node->declare_parameter("track_width", 0.36);
      node->declare_parameter("wheel_radius", 0.1524);

      // Joint names
      node->declare_parameter("front_left_steering_joint",
                              "front_left_steering_joint");
      node->declare_parameter("front_right_steering_joint",
                              "front_right_steering_joint");
      node->declare_parameter("pinion_joint", "pinion_joint");
      node->declare_parameter("back_left_wheel_joint", "back_left_wheel_joint");
      node->declare_parameter("back_right_wheel_joint", "back_right_wheel_joint");
      node->declare_parameter("rear_motor_joint", "rear_motor_joint");

      // Twist subscription
      node->declare_parameter("cmd_vel_topic", "/cmd_vel");
      node->declare_parameter("reference_timeout", 2.0);

      // Calibration parameters
      node->declare_parameter("calibration_sample_size", 1024);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(m_logger, "Exception thrown during init stage with message: %s",
                   e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // MARK: CONF
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {

    auto node = m_node.lock();

    // Get parameters
    // -----------------------------------------------------------------------------
    p_pinion_radius       = node->get_parameter("pinion_radius").as_double();
    p_steering_arm_length = node->get_parameter("steering_arm_length").as_double();
    p_tie_rod_length      = node->get_parameter("tie_rod_length").as_double();
    p_rack_offset_x       = node->get_parameter("rack_offset_x").as_double();
    p_rack_neutral_y      = node->get_parameter("rack_neutral_y").as_double();
    p_pinion_gear_ratio   = node->get_parameter("pinion_gear_ratio").as_double();
    p_max_pinion_angle    = node->get_parameter("max_pinion_angle").as_double();
    p_wheel_angle         = node->get_parameter("wheel_angle").as_double();
    p_wheelbase           = node->get_parameter("wheelbase").as_double();
    p_track_width         = node->get_parameter("track_width").as_double();
    p_wheel_radius        = node->get_parameter("wheel_radius").as_double();

    p_front_left_steering_joint_name =
        node->get_parameter("front_left_steering_joint").as_string();
    p_front_right_steering_joint_name =
        node->get_parameter("front_right_steering_joint").as_string();
    p_pinion_joint_name = node->get_parameter("pinion_joint").as_string();
    p_back_left_wheel_joint_name =
        node->get_parameter("back_left_wheel_joint").as_string();
    p_back_right_wheel_joint_name =
        node->get_parameter("back_right_wheel_joint").as_string();
    p_rear_motor_joint_name = node->get_parameter("rear_motor_joint").as_string();

    p_cmd_vel_topic     = node->get_parameter("cmd_vel_topic").as_string();
    p_reference_timeout = node->get_parameter("reference_timeout").as_double();
    p_calibration_sample_size =
        node->get_parameter("calibration_sample_size").as_int();

    // Initialize calibration
    initialize_calibration();

    // Create twist subscription
    m_twist_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        p_cmd_vel_topic, 10, [this, node](const geometry_msgs::msg::Twist &msg) {
          m_current_twist     = msg;
          m_last_command_time = node->now();
        });

    // Create joint state publisher
    m_joint_state_pub =
        node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    RCLCPP_INFO(
        m_logger,
        "AckermannAngleController configured: pinion_radius=%.5f, wheelbase=%.5f",
        p_pinion_radius, p_wheelbase);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // MARK: ACT
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Get handles to command interfaces by searching through available interfaces
    const std::string pinion_pos_cmd =
        p_pinion_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string rear_motor_vel_cmd =
        p_rear_motor_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

    for (auto &cmd_interface : command_interfaces_) {
      if (cmd_interface.get_name() == pinion_pos_cmd) {
        m_cmd_pinion_pos = &cmd_interface;
      } else if (cmd_interface.get_name() == rear_motor_vel_cmd) {
        m_cmd_rear_motor_vel = &cmd_interface;
      }
    }

    // Get handles to state interfaces
    const std::string pinion_pos_state =
        p_pinion_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string rear_motor_vel_state =
        p_rear_motor_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

    for (const auto &state_interface : state_interfaces_) {
      if (state_interface.get_name() == pinion_pos_state) {
        m_state_pinion_pos = &state_interface;
      } else if (state_interface.get_name() == rear_motor_vel_state) {
        m_state_rear_motor_vel = &state_interface;
      }
    }

    // Initialize position tracking
    m_rear_motor_position       = 0.0;
    m_back_left_wheel_position  = 0.0;
    m_back_right_wheel_position = 0.0;
    m_last_update_time          = rclcpp::Time();

    RCLCPP_INFO(m_logger, "AckermannAngleController activated");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // MARK: DEACT
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Clear handles
    m_cmd_pinion_pos       = nullptr;
    m_cmd_rear_motor_vel   = nullptr;
    m_state_pinion_pos     = nullptr;
    m_state_rear_motor_vel = nullptr;

    RCLCPP_INFO(m_logger, "AckermannAngleController deactivated");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // MARK: CMD I
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration cmd_interfaces_config;
    cmd_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    // COMMAND INTERFACES: Controller writes -> Hardware interface reads
    //
    // Hardware actually controls:
    cmd_interfaces_config.names.push_back(p_pinion_joint_name + "/" +
                                          hardware_interface::HW_IF_POSITION);
    // Single rear motor driving differential (hardware interface drives both wheels
    // from this)
    cmd_interfaces_config.names.push_back(p_rear_motor_joint_name + "/" +
                                          hardware_interface::HW_IF_VELOCITY);

    return cmd_interfaces_config;
  }

  // MARK: STATE I
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    // STATE INTERFACES: Hardware interface writes -> Controller reads
    state_interfaces_config.names.push_back(p_pinion_joint_name + "/" +
                                            hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(p_rear_motor_joint_name + "/" +
                                            hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
  }

  // MARK: UPDATE
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration & /*period*/) override {
    // Check for command timeout
    const double time_since_cmd = (time - m_last_command_time).seconds();
    if (time_since_cmd > p_reference_timeout) {
      // Stop the robot if no command received
      if (m_cmd_rear_motor_vel) { m_cmd_rear_motor_vel->set_value(0.0); }
      return controller_interface::return_type::OK;
    }

    // Get commanded linear velocity (m/s) and ideal angle (rad)
    const double cmd_linear_x = m_current_twist.linear.x;
    const double cmd_ideal_angle =
        m_current_twist.angular.z; // This is ideal angle, not angular velocity!

    // Convert ideal angle to pinion angle using calibration lookup
    const double cmd_pinion_angle = ideal_angle_to_pinion_angle(cmd_ideal_angle);

    // Write pinion angle command
    if (m_cmd_pinion_pos) { m_cmd_pinion_pos->set_value(cmd_pinion_angle); }

    // Convert linear velocity to rear motor velocity (rad/s)
    // linear.x is in m/s, motor velocity = linear_velocity / wheel_radius
    const double cmd_rear_motor_velocity = cmd_linear_x / p_wheel_radius;

    // Write rear motor velocity command
    if (m_cmd_rear_motor_vel) {
      m_cmd_rear_motor_vel->set_value(cmd_rear_motor_velocity);
    }

    // Publish joint states for RViz2 visualization
    // Read pinion position from hardware feedback (or use command if no feedback)
    double actual_pinion_angle = cmd_pinion_angle;
    if (m_state_pinion_pos) {
      actual_pinion_angle = m_state_pinion_pos->get_value();
    }

    // compute wheel angles from actual pinion feedback for accurate visualization
    const auto feedback_linkage_angles = linkage_angles(actual_pinion_angle);
    const double actual_front_left_angle =
        compute_wheel_angle(feedback_linkage_angles.first, true);
    const double actual_front_right_angle =
        compute_wheel_angle(feedback_linkage_angles.second, false);

    // Read rear motor state
    double rear_motor_velocity = cmd_rear_motor_velocity;
    if (m_state_rear_motor_vel) {
      rear_motor_velocity = m_state_rear_motor_vel->get_value();
    }

    // Update wheel positions by integrating velocity
    double dt = 0.0;
    if (m_last_update_time.nanoseconds() > 0) {
      dt = (time - m_last_update_time).seconds();
    }
    m_last_update_time = time;

    if (dt > 0.0 && dt < 1.0) { // Sanity check: dt should be reasonable
      // Integrate motor velocity to get motor position
      m_rear_motor_position += rear_motor_velocity * dt;

      // Both wheels spin at the same rate as the motor [TODO: take in gear ratio]
      // Wheel position in radians
      m_back_left_wheel_position  = m_rear_motor_position;
      m_back_right_wheel_position = m_rear_motor_position;
    }

    // Publish joint states
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = time;
    joint_state_msg.name         = {p_pinion_joint_name,
                                    p_front_left_steering_joint_name,
                                    p_front_right_steering_joint_name,
                                    p_rear_motor_joint_name,
                                    p_back_left_wheel_joint_name,
                                    p_back_right_wheel_joint_name};

    // Positions
    joint_state_msg.position = {
        actual_pinion_angle,        // pinion
        actual_front_left_angle,    // front_left_steering
        actual_front_right_angle,   // front_right_steering
        m_rear_motor_position,      // rear_motor position (integrated from velocity)
        m_back_left_wheel_position, // back_left_wheel (spins with motor)
        m_back_right_wheel_position // back_right_wheel (spins with motor)
    };

    // Velocities
    joint_state_msg.velocity = {
        0.0,                 // pinion velocity (not tracked)
        0.0,                 // front_left_steering velocity (not tracked)
        0.0,                 // front_right_steering velocity (not tracked)
        rear_motor_velocity, // rear_motor velocity
        rear_motor_velocity, // back_left_wheel velocity (assuming same as motor)
        rear_motor_velocity  // back_right_wheel velocity (assuming same as motor)
    };

    m_joint_state_pub->publish(joint_state_msg);

    return controller_interface::return_type::OK;
  }

private:
  // Linkage Parameters MARK: PARAMS
  // -----------------------------------------------------------------------------
  double p_pinion_radius;
  double p_steering_arm_length; // link_a
  double p_tie_rod_length;      // link_b
  double p_rack_offset_x;       // link_c
  double p_rack_neutral_y;      // rack_neutral
  double p_pinion_gear_ratio;
  double p_max_pinion_angle;
  double p_wheel_angle; // wheel angle correction
  double p_wheelbase;
  double p_track_width;
  double p_wheel_radius;

  // Calibration parameters
  int p_calibration_sample_size;

  // Joint names
  // -----------------------------------------------------------------------------
  std::string p_front_left_steering_joint_name;
  std::string p_front_right_steering_joint_name;
  std::string p_pinion_joint_name;
  std::string p_back_left_wheel_joint_name;
  std::string p_back_right_wheel_joint_name;
  std::string p_rear_motor_joint_name;

  // Twist subscription
  // -----------------------------------------------------------------------------
  std::string p_cmd_vel_topic;
  double p_reference_timeout;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_sub;

  geometry_msgs::msg::Twist m_current_twist;
  rclcpp::Time m_last_command_time;

  // Node
  rclcpp_lifecycle::LifecycleNode::WeakPtr m_node;
  // Logger
  rclcpp::Logger m_logger{rclcpp::get_logger("AckermannAngleController")};

  // Joint state publisher for RViz2
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

  // Calibration lookup table (discrete lookup)
  std::vector<double> m_pinion_angles;
  std::vector<double> m_ideal_angles;
  double m_calibration_min;
  double m_calibration_max;
  double m_actual_radius_min;
  double m_actual_radius_max;

  // Command/state interface handles (populated in on_activate)
  hardware_interface::LoanedCommandInterface *m_cmd_pinion_pos     = nullptr;
  hardware_interface::LoanedCommandInterface *m_cmd_rear_motor_vel = nullptr;

  const hardware_interface::LoanedStateInterface *m_state_pinion_pos     = nullptr;
  const hardware_interface::LoanedStateInterface *m_state_rear_motor_vel = nullptr;

  // Position tracking for wheels (integrated from velocity)
  double m_rear_motor_position       = 0.0;
  double m_back_left_wheel_position  = 0.0;
  double m_back_right_wheel_position = 0.0;
  rclcpp::Time m_last_update_time;

  // Helper methods
  // -----------------------------------------------------------------------------
  // MARK: CALIBRATE
  void initialize_calibration() {
    // Build calibration lookup table with discrete steps
    // Similar to Python build_calibration, but creates discrete mapping
    m_pinion_angles.reserve(p_calibration_sample_size);
    m_ideal_angles.reserve(p_calibration_sample_size);
    std::vector<double> actual_radii;
    actual_radii.reserve(p_calibration_sample_size);

    // Generate discrete pinion angles from -max to +max
    for (int i = 0; i < p_calibration_sample_size; ++i) {
      const double pinion_angle =
          -p_max_pinion_angle +
          (2.0 * p_max_pinion_angle * i) / (p_calibration_sample_size - 1);
      m_pinion_angles.push_back(pinion_angle);

      // Compute linkage angles for this pinion angle
      const auto angles = linkage_angles(pinion_angle);

      // Compute actual wheel angles (theta_2 is left, theta_4 is right)
      const double theta_left  = compute_wheel_angle(angles.first, true);
      const double theta_right = compute_wheel_angle(angles.second, false);

      // Compute actual turning radius
      const double r_actual =
          p_wheelbase / (2.0 * (tan(theta_left) + tan(theta_right)));
      actual_radii.push_back(r_actual);

      // Compute ideal angle from actual radius
      const double ideal_angle = atan(p_wheelbase / r_actual);
      m_ideal_angles.push_back(ideal_angle);
    }

    // Store calibration range
    m_calibration_min =
        *std::min_element(m_ideal_angles.begin(), m_ideal_angles.end());
    m_calibration_max =
        *std::max_element(m_ideal_angles.begin(), m_ideal_angles.end());
    m_actual_radius_min =
        *std::min_element(actual_radii.begin(), actual_radii.end());
    m_actual_radius_max =
        *std::max_element(actual_radii.begin(), actual_radii.end());

    RCLCPP_INFO(m_logger,
                "Calibration initialized with %d samples, ideal angle range: [%.4f, "
                "%.4f], actual radius range: [%.4f, %.4f]",
                p_calibration_sample_size, m_calibration_min, m_calibration_max,
                m_actual_radius_min, m_actual_radius_max);
  }

  // MARK: ANGLE
  double ideal_angle_to_pinion_angle(double ideal_angle) {
    // Clamp ideal angle to calibration range
    ideal_angle = std::clamp(ideal_angle, m_calibration_min, m_calibration_max);

    // Find closest registered ideal angle (discrete lookup, no interpolation)
    double min_diff    = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < m_ideal_angles.size(); ++i) {
      const double diff = std::abs(m_ideal_angles[i] - ideal_angle);
      if (diff < min_diff) {
        min_diff    = diff;
        closest_idx = i;
      }
    }

    // Return the pinion angle corresponding to the closest ideal angle
    return m_pinion_angles[closest_idx];
  }

  // MARK: LINKAGE
  std::pair<double, double> linkage_angles(double pinion_angle) {
    // Ported from Python Linkage_Angles function
    const double a = p_steering_arm_length; // link_a
    const double b = p_tie_rod_length;      // link_b
    const double c = p_rack_offset_x;       // link_c
    const double d = p_rack_neutral_y + (pinion_angle * p_pinion_gear_ratio *
                                         p_pinion_radius); // rack position

    // Helper function for slider-crank theta2
    auto slider_crank_theta2 = [](double K_1, double K_2, double K_3) -> double {
      const double A            = K_1 - K_3;
      const double B            = 2.0 * K_2;
      const double C            = K_1 + K_3;
      const double discriminant = B * B - 4.0 * A * C;
      if (discriminant < 0) {
        return 0.0; // Invalid case
      }
      const double theta_2 = 2.0 * atan((-B + sqrt(discriminant)) / (2.0 * A));
      return theta_2;
    };

    const double K_1     = a * a - b * b + c * c + d * d;
    const double K_2     = -2.0 * a * c;
    const double K_3     = -2.0 * a * d;
    const double theta_2 = slider_crank_theta2(K_1, K_2, K_3);

    const double e       = a;
    const double f       = b;
    const double g       = c;
    const double h       = (p_rack_neutral_y - d) + p_rack_neutral_y;
    const double K_4     = e * e - f * f + g * g + h * h;
    const double K_5     = -2.0 * e * g;
    const double K_6     = -2.0 * e * h;
    const double theta_4 = -(slider_crank_theta2(K_4, K_5, K_6) + M_PI);

    return {theta_2, theta_4};
  }

  double compute_wheel_angle(double linkage_angle, bool is_left) {
    // Convert linkage angle to wheel angle
    // From Python: theta_left = angles[0] + pi + wheel_angle
    //              theta_right = angles[2] - wheel_angle
    if (is_left) {
      return linkage_angle + M_PI + p_wheel_angle;
    } else {
      return linkage_angle - p_wheel_angle;
    }
  }
};

} // namespace igvc_control

// MARK: PLUGIN
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(igvc_control::AckermannAngleController,
                       controller_interface::ControllerInterface)