#include <cmath>
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

      // Initialize command and state storage
      m_pinion_position_cmd_       = 0.0;
      m_rear_motor_velocity_cmd_   = 0.0;
      m_pinion_position_state_     = 0.0;
      m_rear_motor_velocity_state_ = 0.0;

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

  // MARK: ACT
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Open serial port
    m_serial_fd = open(p_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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
  return_type read(const rclcpp::Time & /*time*/,
                   const rclcpp::Duration & /*period*/) override {
    if (m_serial_fd < 0) {
      // No serial connection, use command values as fallback
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      return return_type::OK;
    }

    // Read available data from serial port
    char read_buffer[256];
    ssize_t bytes_read = ::read(m_serial_fd, read_buffer, sizeof(read_buffer) - 1);

    if (bytes_read > 0) {
      read_buffer[bytes_read] = '\0'; // Null terminate

      // Parse feedback messages
      // Expected format: "F:P:<pinion_angle>,V:<velocity>\n"
      std::string feedback(read_buffer);
      parseFeedback(feedback);
    } else {
      // No data available, use command values as fallback
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
    }

    return return_type::OK;
  }

private:
  // MARK: FEEDBACK
  // -----------------------------------------------------------------------------
  void parseFeedback(const std::string &feedback) {
    // Look for feedback format: "F:P:<angle>,V:<velocity>"
    size_t f_pos = feedback.find("F:P:");
    if (f_pos == std::string::npos) {
      // Not a feedback message, use command values
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      return;
    }

    size_t v_pos = feedback.find(",V:", f_pos);
    if (v_pos == std::string::npos) {
      // Invalid format, use command values
      m_pinion_position_state_     = m_pinion_position_cmd_;
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
      return;
    }

    // Extract pinion angle
    std::string pinion_str = feedback.substr(f_pos + 4, v_pos - (f_pos + 4));
    try {
      m_pinion_position_state_ = std::stod(pinion_str);
    } catch (const std::exception &) {
      // Parse error, use command value
      m_pinion_position_state_ = m_pinion_position_cmd_;
    }

    // Extract velocity
    size_t end_pos = feedback.find('\n', v_pos);
    if (end_pos == std::string::npos) { end_pos = feedback.length(); }
    std::string velocity_str = feedback.substr(v_pos + 3, end_pos - (v_pos + 3));
    try {
      m_rear_motor_velocity_state_ = std::stod(velocity_str);
    } catch (const std::exception &) {
      // Parse error, use command value
      m_rear_motor_velocity_state_ = m_rear_motor_velocity_cmd_;
    }
  }

  // MARK: WRITE
  return_type write(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) override {
    if (m_serial_fd < 0) { return return_type::ERROR; }

    // Send commands to Arduino
    // Protocol: "P:<pinion_angle>,V:<velocity>\n"
    // Pinion angle in radians, velocity in rad/s
    char cmd_buffer[128];
    int len = snprintf(cmd_buffer, sizeof(cmd_buffer), "P:%.6f,V:%.6f\n",
                       m_pinion_position_cmd_, m_rear_motor_velocity_cmd_);

    if (len > 0 && len < static_cast<int>(sizeof(cmd_buffer))) {
      ssize_t written = ::write(m_serial_fd, cmd_buffer, len);
      if (written != len) {
        RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 1000,
                             "Failed to write complete command to serial port");
      }
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
