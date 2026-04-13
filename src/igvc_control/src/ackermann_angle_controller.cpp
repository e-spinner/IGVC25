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

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace igvc_control {

constexpr double MAX_LINEAR_SPEED_MPS = 2.2352; // 5 mph

class AckermannAngleController final
    : public controller_interface::ControllerInterface {
public:
  AckermannAngleController() : controller_interface::ControllerInterface() {}

  // MARK: INIT
  controller_interface::CallbackReturn on_init() override {
        m_node   = get_node();
    auto node = m_node.lock();
    m_logger = node->get_logger();

    auto declare = [&](const std::string& name, auto value) {
        if (!node->has_parameter(name)) {
            node->declare_parameter(name, value);
        }
    };

    declare("pinion_radius",              0.01905);
    declare("steering_arm_length",        0.0381);
    declare("tie_rod_length",             0.127);
    declare("rack_offset_x",              -0.0315);
    declare("rack_neutral_y",             0.131064);
    declare("pinion_gear_ratio",          1.652);
    declare("max_pinion_angle",           2.0);
    declare("wheel_angle",                0.32253);
    declare("wheelbase",                  0.42);
    declare("track_width",                0.36);
    declare("wheel_radius",               0.1524);
    declare("pinion_joint",               std::string("pinion_joint"));
    declare("motor_joint",                std::string("motor_joint"));
    declare("front_left_steering_joint",  std::string("front_left_steering_joint"));
    declare("front_right_steering_joint", std::string("front_right_steering_joint"));
    declare("back_left_wheel_joint",      std::string("back_left_wheel_joint"));
    declare("back_right_wheel_joint",     std::string("back_right_wheel_joint"));
    declare("cmd_vel_topic",              std::string("/cmd_vel_unstamped"));
    declare("reference_timeout",          2.0);
    declare("calibration_sample_size",    1024);

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
    // Single rear motor driving differential
    cmd_interfaces_config.names.push_back(p_motor_joint_name + "/" +
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
    state_interfaces_config.names.push_back(p_motor_joint_name + "/" +
                                            hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(p_motor_joint_name + "/" +
                                            hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
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
    p_motor_joint_name = node->get_parameter("motor_joint").as_string();

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
    m_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/raw", 10);
    m_last_odom_time = node->now();

    RCLCPP_INFO(
        m_logger,
        "AckermannAngleController configured: pinion_radius=%.5f, wheelbase=%.5f",
        p_pinion_radius, p_wheelbase);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  // MARK: ACT
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    auto node = m_node.lock();

    // Get handles to command interfaces by searching through available interfaces
    const std::string pinion_pos_cmd =
        p_pinion_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string rear_motor_vel_cmd =
        p_motor_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

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
        p_motor_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

    for (const auto &state_interface : state_interfaces_) {
      if (state_interface.get_name() == pinion_pos_state) {
        m_state_pinion_pos = &state_interface;
      } else if (state_interface.get_name() == rear_motor_vel_state) {
        m_state_rear_motor_vel = &state_interface;
      }
    }

    // Initialize last command time to current time to avoid time source mismatch
    // This ensures m_last_command_time uses the same clock source as update() calls
    m_last_command_time = node->now();

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

    // Get commanded linear velocity (m/s) and ideal steering angle (rad)
    const double cmd_linear_x =
        std::clamp(m_current_twist.linear.x, -MAX_LINEAR_SPEED_MPS,
                   MAX_LINEAR_SPEED_MPS);
    const double cmd_ideal_angle =
        std::clamp(m_current_twist.angular.z, -p_max_pinion_angle,
                   p_max_pinion_angle);

    // Convert ideal angle to pinion angle using calibration lookup
    // const double cmd_pinion_angle = ideal_angle_to_pinion_angle(cmd_ideal_angle);

    // Write pinion angle command
    // TODO: passing over calibration for now
    // if (m_cmd_pinion_pos) { m_cmd_pinion_pos->set_value(cmd_pinion_angle); }
    if (m_cmd_pinion_pos) { m_cmd_pinion_pos->set_value(cmd_ideal_angle); }

    // Convert linear velocity to rear motor velocity (rad/s)
    // linear.x is in m/s, motor velocity = linear_velocity / wheel_radius
    const double cmd_rear_motor_velocity = cmd_linear_x / p_wheel_radius;

    // Write rear motor velocity command
    if (m_cmd_rear_motor_vel) {
      m_cmd_rear_motor_vel->set_value(cmd_rear_motor_velocity);
    }

    update_odometry(time);

    return controller_interface::return_type::OK;
  }

  void update_odometry(const rclcpp::Time &time) {
    if (!m_state_pinion_pos || !m_state_rear_motor_vel) return;

    double dt = (time - m_last_odom_time).seconds();
    if (dt <= 0.0) return;

    // 1. Get States
    double pinion_pos = m_state_pinion_pos->get_value();
    double motor_vel  = m_state_rear_motor_vel->get_value(); // rad/s

    // 2. Convert to linear velocity and steering angle
    double v = motor_vel * p_wheel_radius;

    // Use your linkage logic to find the average wheel angle (delta)
    auto angles = linkage_angles(pinion_pos);
    double theta_l = compute_wheel_angle(angles.first, true);
    double theta_r = compute_wheel_angle(angles.second, false);
    double delta = (theta_l + theta_r) / 2.0;

    // 3. Update Pose (Dead Reckoning)
    m_pose_x   += v * cos(m_pose_yaw) * dt;
    m_pose_y   += v * sin(m_pose_yaw) * dt;
    m_pose_yaw += (v / p_wheelbase) * tan(delta) * dt;
    m_last_odom_time = time;

    // 4. Create and Publish Message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = m_pose_x;
    odom_msg.pose.pose.position.y = m_pose_y;

    // Set covariance (diagonal values for X, Y, Z, Roll, Pitch, Yaw)
    // Position covariance
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0]  = 0.05; // X
    odom_msg.pose.covariance[7]  = 0.05; // Y
    odom_msg.pose.covariance[35] = 0.1;  // Yaw

    // Twist (Velocity) covariance
    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0]  = 0.01; // Linear X
    odom_msg.twist.covariance[35] = 0.05; // Angular Z

    tf2::Quaternion q;
    q.setRPY(0, 0, m_pose_yaw);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = (v / p_wheelbase) * tan(delta);

    m_odom_pub->publish(odom_msg);
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
  std::string p_motor_joint_name;

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

  // Odometry State
  double m_pose_x = 0.0;
  double m_pose_y = 0.0;
  double m_pose_yaw = 0.0;
  rclcpp::Time m_last_odom_time;

  // Standard ROS 2 Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

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