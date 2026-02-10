#include <cmath>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace igvc_control {

class AckermannArduinoHardware final : public hardware_interface::SystemInterface {
public:
  AckermannArduinoHardware() : hardware_interface::SystemInterface() {}

  // MARK: INIT
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    try {
      if (hardware_interface::SystemInterface::on_init(info) !=
          CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
      }

      // Get parameters from hardware info
      // -----------------------------------------------------------------------------

      // Serial communication parameters
      if (info_.hardware_parameters.find("device") ==
          info_.hardware_parameters.end()) {
        RCLCPP_ERROR(m_logger, "Missing required parameter: device");
        return CallbackReturn::ERROR;
      }
      p_device = info_.hardware_parameters.at("device");

      if (info_.hardware_parameters.find("baud_rate") ==
          info_.hardware_parameters.end()) {
        RCLCPP_ERROR(m_logger, "Missing required parameter: baud_rate");
        return CallbackReturn::ERROR;
      }
      p_baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

      if (info_.hardware_parameters.find("timeout_ms") ==
          info_.hardware_parameters.end()) {
        RCLCPP_ERROR(m_logger, "Missing required parameter: timeout_ms");
        return CallbackReturn::ERROR;
      }
      p_timeout_ms = std::stoi(info_.hardware_parameters.at("timeout_ms"));

      // Joint names
      if (info_.hardware_parameters.find("pinion_joint_name") ==
          info_.hardware_parameters.end()) {
        RCLCPP_ERROR(m_logger, "Missing required parameter: pinion_joint_name");
        return CallbackReturn::ERROR;
      }
      p_pinion_joint_name = info_.hardware_parameters.at("pinion_joint_name");

      if (info_.hardware_parameters.find("rear_motor_joint_name") ==
          info_.hardware_parameters.end()) {
        RCLCPP_ERROR(m_logger, "Missing required parameter: rear_motor_joint_name");
        return CallbackReturn::ERROR;
      }
      p_rear_motor_joint_name =
          info_.hardware_parameters.at("rear_motor_joint_name");

      // Linkage parameters (optional, with defaults)
      p_steering_arm_length =
          info_.hardware_parameters.find("steering_arm_length") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("steering_arm_length"))
              : 0.0381;
      p_tie_rod_length =
          info_.hardware_parameters.find("tie_rod_length") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("tie_rod_length"))
              : 0.127;
      p_rack_offset_x =
          info_.hardware_parameters.find("rack_offset_x") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("rack_offset_x"))
              : -0.0315;
      p_rack_neutral_y =
          info_.hardware_parameters.find("rack_neutral_y") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("rack_neutral_y"))
              : 0.131064;
      p_pinion_gear_ratio =
          info_.hardware_parameters.find("pinion_gear_ratio") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("pinion_gear_ratio"))
              : 1.652;
      p_pinion_radius =
          info_.hardware_parameters.find("pinion_radius") !=
                  info_.hardware_parameters.end()
              ? std::stod(info_.hardware_parameters.at("pinion_radius"))
              : 0.01905;
      p_wheel_angle = info_.hardware_parameters.find("wheel_angle") !=
                              info_.hardware_parameters.end()
                          ? std::stod(info_.hardware_parameters.at("wheel_angle"))
                          : 0.32253;

      // Initialize command and state storage
      m_pinion_position_cmd_       = 0.0;
      m_rear_motor_velocity_cmd_   = 0.0;
      m_pinion_position_state_     = 0.0;
      m_rear_motor_velocity_state_ = 0.0;

      // Derived joint states
      m_front_left_steering_position_  = 0.0;
      m_front_right_steering_position_ = 0.0;
      m_back_left_wheel_position_      = 0.0;
      m_back_left_wheel_velocity_      = 0.0;
      m_back_right_wheel_position_     = 0.0;
      m_back_right_wheel_velocity_     = 0.0;

      // Position integration tracking
      m_rear_motor_position_ = 0.0;
      m_last_read_time_      = rclcpp::Time();

      m_serial_fd = -1;

    } catch (const std::exception &e) {
      RCLCPP_ERROR(m_logger, "Exception thrown during init stage with message: %s",
                   e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(m_logger, "AckermannArduinoHardware initialized: device=%s, baud=%d",
                p_device.c_str(), p_baud_rate);

    return CallbackReturn::SUCCESS;
  }

  // MARK: STATE I
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // STATE INTERFACES: Hardware interface writes → Controller reads
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        p_pinion_joint_name, hardware_interface::HW_IF_POSITION,
        &m_pinion_position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        p_rear_motor_joint_name, hardware_interface::HW_IF_VELOCITY,
        &m_rear_motor_velocity_state_));

    // Derived joint states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_left_steering_joint", hardware_interface::HW_IF_POSITION,
        &m_front_left_steering_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_right_steering_joint", hardware_interface::HW_IF_POSITION,
        &m_front_right_steering_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "back_left_wheel_joint", hardware_interface::HW_IF_POSITION,
        &m_back_left_wheel_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "back_left_wheel_joint", hardware_interface::HW_IF_VELOCITY,
        &m_back_left_wheel_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "back_right_wheel_joint", hardware_interface::HW_IF_POSITION,
        &m_back_right_wheel_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "back_right_wheel_joint", hardware_interface::HW_IF_VELOCITY,
        &m_back_right_wheel_velocity_));

    return state_interfaces;
  }

  // MARK: CMD I
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // COMMAND INTERFACES: Controller writes → Hardware interface reads
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        p_pinion_joint_name, hardware_interface::HW_IF_POSITION,
        &m_pinion_position_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        p_rear_motor_joint_name, hardware_interface::HW_IF_VELOCITY,
        &m_rear_motor_velocity_cmd_));

    return command_interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & /*start_interfaces*/,
      const std::vector<std::string> & /*stop_interfaces*/) override {

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & /*start_interfaces*/,
      const std::vector<std::string> & /*stop_interfaces*/) override {

    return hardware_interface::return_type::OK;
  }

  // MARK: ACT
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Open serial port
    m_serial_fd = open(p_device.c_str(), O_RDWR | O_NOCTTY);
    if (m_serial_fd == -1) {
      RCLCPP_ERROR(m_logger, "Failed to open serial device: %s", p_device.c_str());
      return CallbackReturn::ERROR;
    }

    // Configure serial port
    struct termios tty;
    if (tcgetattr(m_serial_fd, &tty) != 0) {
      RCLCPP_ERROR(m_logger, "Failed to get serial attributes");
      close(m_serial_fd);
      m_serial_fd = -1;
      return CallbackReturn::ERROR;
    }

    // Set baud rate
    speed_t baud = B9600; // Default to 9600
    switch (p_baud_rate) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    default:
      RCLCPP_WARN(m_logger, "Unsupported baud rate %d, using 9600", p_baud_rate);
      baud = B9600;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // Configure serial port settings
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Clear size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore modem controls

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, DEL

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable special handling of received bytes

    tty.c_oflag &= ~OPOST; // Disable post-processing
    tty.c_oflag &=
        ~ONLCR; // Disable conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = p_timeout_ms / 100; // Timeout in tenths of seconds
    tty.c_cc[VMIN]  = 0;                  // Non-blocking read

    if (tcsetattr(m_serial_fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(m_logger, "Failed to set serial attributes");
      close(m_serial_fd);
      m_serial_fd = -1;
      return CallbackReturn::ERROR;
    }

    // Flush any existing data
    tcflush(m_serial_fd, TCIOFLUSH);

    // Initialize time tracking
    m_last_read_time_      = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    m_rear_motor_position_ = 0.0;

    RCLCPP_INFO(m_logger, "AckermannArduinoHardware activated");
    return CallbackReturn::SUCCESS;
  }

  // MARK: DEACT
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    if (m_serial_fd >= 0) {
      close(m_serial_fd);
      m_serial_fd = -1;
      RCLCPP_INFO(m_logger, "AckermannArduinoHardware deactivated");
    }
    return CallbackReturn::SUCCESS;
  }

  // MARK: READ
  return_type read(const rclcpp::Time &time,
                   const rclcpp::Duration & /*period*/) override {
    if (m_serial_fd < 0) {
      // No serial connection, use command values as fallback
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      RCLCPP_DEBUG(m_logger,
                   "READ: No serial connection, using command values as fallback");
    } else {
      // Read available data from serial port
      char read_buffer[256];
      ssize_t bytes_read = ::read(m_serial_fd, read_buffer, sizeof(read_buffer) - 1);

      if (bytes_read > 0) {
        read_buffer[bytes_read] = '\0'; // Null terminate

        // Log raw data received
        std::string raw_data(read_buffer, bytes_read);
        RCLCPP_INFO(m_logger, "READ from Arduino: [%zd bytes] '%s'", bytes_read,
                    raw_data.c_str());

        // Parse feedback messages
        // Expected format: "F:P:<pinion_angle>,V:<velocity>\n"
        std::string feedback(read_buffer);
        double pinion_before   = m_pinion_position_state_;
        double velocity_before = m_rear_motor_velocity_state_;
        parseFeedback(feedback);

        // Log parsed values
        if (m_pinion_position_state_ != pinion_before ||
            m_rear_motor_velocity_state_ != velocity_before) {
          RCLCPP_INFO(m_logger, "READ parsed: pinion=%.6f, velocity=%.6f",
                      m_pinion_position_state_, m_rear_motor_velocity_state_);
        }
      } else if (bytes_read == 0) {
        // No data available, use command values as fallback
        m_pinion_position_state_     = m_pinion_position_cmd_;
        m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
        RCLCPP_DEBUG(m_logger,
                     "READ: No data available from Arduino, using command values");
      } else {
        // Error reading
        RCLCPP_WARN(m_logger, "READ error: %s", strerror(errno));
      }
    }

    if (m_last_read_time_.nanoseconds() <= 0) {
      m_last_read_time_ = time;
      return return_type::OK;
    }

    // Compute derived joint states from pinion and motor states
    computeDerivedStates(time);

    return return_type::OK;
  }

private:
  // MARK: DERIVED STATES
  // -----------------------------------------------------------------------------
  void computeDerivedStates(const rclcpp::Time &time) {
    // Compute steering angles from pinion angle using linkage calculation
    const auto linkage_angles       = computeLinkageAngles(m_pinion_position_state_);
    m_front_left_steering_position_ = linkage_angles.first + M_PI + p_wheel_angle;
    m_front_right_steering_position_ = linkage_angles.second - p_wheel_angle;

    // Compute wheel positions and velocities from motor velocity
    // Integrate motor velocity to get position
    double dt = 0.0;
    if (m_last_read_time_.nanoseconds() > 0) {
      dt = (time - m_last_read_time_).seconds();
    }
    m_last_read_time_ = time;

    if (dt > 0.0 && dt < 1.0) { // Sanity check
      m_rear_motor_position_ += m_rear_motor_velocity_state_ * dt;
    }

    // Both wheels spin at the same rate as the motor
    m_back_left_wheel_position_  = m_rear_motor_position_;
    m_back_right_wheel_position_ = m_rear_motor_position_;
    m_back_left_wheel_velocity_  = m_rear_motor_velocity_state_;
    m_back_right_wheel_velocity_ = m_rear_motor_velocity_state_;
  }

  std::pair<double, double> computeLinkageAngles(double pinion_angle) {
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

  // MARK: FEEDBACK
  // -----------------------------------------------------------------------------
  void parseFeedback(const std::string &feedback) {
    // Look for feedback format: "F:P:<angle>,V:<velocity>"
    size_t f_pos = feedback.find("F:P:");
    if (f_pos == std::string::npos) {
      // Not a feedback message, use command values
      RCLCPP_WARN(m_logger, "PARSE: Feedback message missing 'F:P:' marker in: '%s'",
                  feedback.c_str());
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      return;
    }

    size_t v_pos = feedback.find(",V:", f_pos);
    if (v_pos == std::string::npos) {
      // Invalid format, use command values
      RCLCPP_WARN(m_logger, "PARSE: Feedback message missing ',V:' marker in: '%s'",
                  feedback.c_str());
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      return;
    }

    // Extract pinion angle
    std::string pinion_str = feedback.substr(f_pos + 4, v_pos - (f_pos + 4));
    try {
      m_pinion_position_state_ = std::stod(pinion_str);
    } catch (const std::exception &e) {
      // Parse error, use command value
      RCLCPP_WARN(m_logger, "PARSE: Failed to parse pinion angle from '%s': %s",
                  pinion_str.c_str(), e.what());
      m_pinion_position_state_ = m_pinion_position_cmd_;
    }

    // Extract velocity
    size_t end_pos = feedback.find('\n', v_pos);
    if (end_pos == std::string::npos) { end_pos = feedback.length(); }
    std::string velocity_str = feedback.substr(v_pos + 3, end_pos - (v_pos + 3));
    try {
      m_rear_motor_velocity_state_ = std::stod(velocity_str);
    } catch (const std::exception &e) {
      // Parse error, use command value
      RCLCPP_WARN(m_logger, "PARSE: Failed to parse velocity from '%s': %s",
                  velocity_str.c_str(), e.what());
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
    }
  }

  // MARK: WRITE
  return_type write(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) override {
    if (m_serial_fd < 0) {
      RCLCPP_WARN(m_logger, "WRITE: No serial connection available");
      return return_type::ERROR;
    }

    // Construct the string: "P:0.123,V:1.23\n"
    std::string msg = "P:" + std::to_string(m_pinion_position_cmd_) +
                      ",V:" + std::to_string(m_rear_motor_velocity_cmd_) + "\n";

    // Log what we're sending
    RCLCPP_INFO(m_logger, "WRITE to Arduino: pinion=%.6f, velocity=%.6f -> '%s'",
                m_pinion_position_cmd_, m_rear_motor_velocity_cmd_, msg.c_str());

    ssize_t bytes_written = ::write(m_serial_fd, msg.c_str(), msg.length());

    if (bytes_written < 0) {
      RCLCPP_WARN(m_logger, "WRITE failed: %s", strerror(errno));
      return return_type::ERROR;
    }

    if (bytes_written != static_cast<ssize_t>(msg.length())) {
      RCLCPP_WARN(m_logger, "WRITE incomplete: wrote %zd of %zu bytes",
                  bytes_written, msg.length());
    } else {
      RCLCPP_DEBUG(m_logger, "WRITE successful: %zd bytes written", bytes_written);
    }

    return return_type::OK;
  }

private:
  // Serial communication parameters
  // -----------------------------------------------------------------------------
  std::string p_device;
  int p_baud_rate;
  int p_timeout_ms;

  // Joint names
  // -----------------------------------------------------------------------------
  std::string p_pinion_joint_name;
  std::string p_rear_motor_joint_name;

  // Linkage parameters
  // -----------------------------------------------------------------------------
  double p_steering_arm_length;
  double p_tie_rod_length;
  double p_rack_offset_x;
  double p_rack_neutral_y;
  double p_pinion_gear_ratio;
  double p_pinion_radius;
  double p_wheel_angle;

  // Serial communication
  // -----------------------------------------------------------------------------
  int m_serial_fd = -1;

  // Command storage
  // -----------------------------------------------------------------------------
  double m_pinion_position_cmd_;
  double m_rear_motor_velocity_cmd_;

  // State storage
  // -----------------------------------------------------------------------------
  double m_pinion_position_state_;
  double m_rear_motor_velocity_state_;

  // Derived joint states
  // -----------------------------------------------------------------------------
  double m_front_left_steering_position_;
  double m_front_right_steering_position_;
  double m_back_left_wheel_position_;
  double m_back_left_wheel_velocity_;
  double m_back_right_wheel_position_;
  double m_back_right_wheel_velocity_;

  // Position integration tracking
  double m_rear_motor_position_;
  rclcpp::Time m_last_read_time_;

  // Logger
  rclcpp::Logger m_logger{rclcpp::get_logger("AckermannArduinoHardware")};

  // Clock for throttled logging
  rclcpp::Clock::SharedPtr m_clock{std::make_shared<rclcpp::Clock>()};
};

} // namespace igvc_control

// MARK: PLUGIN
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(igvc_control::AckermannArduinoHardware,
                       hardware_interface::SystemInterface)
