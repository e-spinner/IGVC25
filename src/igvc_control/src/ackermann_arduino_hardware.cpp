#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <sys/types.h>
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
          CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

      // get params
      p_device       = info_.hardware_parameters.at("device");
      p_baud_rate    = info_.hardware_parameters.at("baud_rate");
      p_pinion_joint = info_.hardware_parameters.at("pinion_joint");
      p_motor_joint  = info_.hardware_parameters.at("motor_joint");

      // init storage
      m_motor_vel_cmd    = 0.0d;
      m_motor_vel_state  = 0.0d;
      m_motor_pos_state  = 0.0d;
      m_pinion_pos_cmd   = 0.0d;
      m_pinion_pos_state = 0.0d;

      // init position integration
      m_last_read_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

      // init serial
      m_serial_fd     = -1;
      m_serial_buffer = "";

    } catch (const std::exception &e) {
      RCLCPP_ERROR(m_logger, "Exception during init: %s", e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(m_logger,
                "AckermannArduinoHardware initailized: device[%s], baud[%d]",
                p_device.c_str(), p_baud_rate);
    return CallbackReturn::SUCCESS;
  }

  // MARK: STATE I
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Hardware writes -> controller reads
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        p_pinion_joint, hardware_interface::HW_IF_POSITION, &m_pinion_pos_state));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        p_motor_joint, hardware_interface::HW_IF_VELOCITY, &m_motor_vel_state));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        p_motor_joint, hardware_interface::HW_IF_POSITION, &m_motor_pos_state));

    return state_interfaces;
  }

  // MARK: CMD I
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Controller writes -> hardware reads
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        p_pinion_joint, hardware_interface::HW_IF_POSITION, &m_pinion_pos_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        p_motor_joint, hardware_interface::HW_IF_VELOCITY, &m_motor_vel_cmd));

    return command_interfaces;
  }

  // MARK: ACT
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // open serial port
    m_serial_fd = open(p_device.c_str(), O_RDWR | O_NOCTTY);
    if (m_serial_fd == -1) {
      RCLCPP_ERROR(m_logger, "Failed to open serial device: %s", p_device.c_str());
      return CallbackReturn::ERROR;
    }

    // config serial
    struct termios tty;
    if (tcgetattr(m_serial_fd, &tty) != 0) {
      RCLCPP_ERROR(m_logger, "Failed to get serial attributes!");
      close(m_serial_fd);
      m_serial_fd = -1;
      return CallbackReturn::ERROR;
    }

    // Set baud rate
    speed_t baud = B9600;
    switch (p_baud_rate) {
    case 9600: break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    default:
    }

    RCLCPP_INFO(m_logger, "Serial opened using baud[%i]", baud);

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

    m_serial_buffer = ""; // Clear serial buffer

    return CallbackReturn::SUCCESS;
  }

  // MARK: DEACT
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    if (m_serial_fd >= 0) {
      close(m_serial_fd);
      m_serial_fd = -1;
      RCLCPP_INFO(m_logger, "Serial Connection closed");
    }
    return CallbackReturn::SUCCESS;
  }

  // MARK: READ
  return_type read(const rclcpp::Time &time,
                   const rclcpp::Duration &period) override {
    // Integrate motor position from velocity
    if (m_last_read_time.nanoseconds() > 0) {
      // Calculate time delta in seconds
      double dt = period.seconds();
      // Integrate velocity to get position: position += velocity * dt
      m_motor_pos_state += m_motor_vel_state * dt;
    }
    m_last_read_time = time;

    if (m_serial_fd < 0)
      RCLCPP_WARN(m_logger, "No serial connection, Cannot read!");
    else {
      // read available data from port and accumulate in buffer
      char read_buffer[256];
      ssize_t bytes_read = ::read(m_serial_fd, read_buffer, sizeof(read_buffer) - 1);

      if (bytes_read > 0) {
        read_buffer[bytes_read] = '\0'; // terminate message

        // log for debug TODO: Stop doing this
        RCLCPP_DEBUG(m_logger, "%s", std::string(read_buffer, bytes_read).c_str());

        // accumulate new data
        m_serial_buffer += std::string(read_buffer, bytes_read);

        // Proccess all complete lines in buffer
        size_t newline_pos;
        while ((newline_pos = m_serial_buffer.find('\n')) != std::string::npos) {

          std::string complete_line = m_serial_buffer.substr(0, newline_pos);
          m_serial_buffer.erase(0, newline_pos + 1);

          parse_feedback(complete_line);

          if (m_serial_buffer.length() > 512) {
            RCLCPP_WARN(m_logger, "Serial buffer overflow, oof!");
            m_serial_buffer.clear();
          }
        }
      } else if (bytes_read == 0) {
        RCLCPP_WARN(m_logger, "no data available over serial!");
      } else
        RCLCPP_WARN(m_logger, "Read error: %s", strerror(errno));

      return return_type::OK;
    }
  }

  // MARK: WRITE
  return_type write(const rclcpp::Time & /*time*/,
                    const rclcpp::Duration & /*period*/) override {
    if (m_serial_fd < 0) {
      RCLCPP_WARN(m_logger, "No serial device available!");
      return return_type::ERROR;
    }

    std::string msg = "P:" + std::to_string(m_pinion_pos_cmd) +
                      ",V:" + std::to_string(m_motor_vel_cmd) + "\n";

    ssize_t bytes_written = ::write(m_serial_fd, msg.c_str(), msg.length());

    if (bytes_written < 0) {
      RCLCPP_WARN(m_logger, "Write failed: %s", strerror(errno));
      return return_type::ERROR;
    }

    if (bytes_written != static_cast<ssize_t>(msg.length())) {
      RCLCPP_WARN(m_logger, "Write incomplete!");
    }

    return return_type::OK;
  }

private:
  // Serial communication
  std::string p_device;
  int p_baud_rate;
  int p_timeout_ms;

  int m_serial_fd;
  std::string m_serial_buffer;

  // Joint names
  std::string p_pinion_joint;
  std::string p_motor_joint;

  // Command storage
  double m_pinion_pos_cmd;
  double m_motor_vel_cmd;

  // State storage
  double m_pinion_pos_state;
  double m_motor_vel_state;
  double m_motor_pos_state;

  // Position integration
  rclcpp::Time m_last_read_time;

  // Logging
  rclcpp::Logger m_logger{rclcpp::get_logger("AckermannArduinoHardware")};

  // MARK: FEEDBACK
  // -----------------------------------------------------------------------------
  void parse_feedback(const std::string &feedback) {
    // Look for feedback format: "F:P:<angle>,V:<velocity>"
    size_t f_pos = feedback.find("F:P:");
    if (f_pos == std::string::npos) {
      // Not a feedback message, use command values
      RCLCPP_WARN(m_logger, "PARSE: Feedback message missing 'F:P:' marker in: '%s'",
                  feedback.c_str());
      m_pinion_pos_state = m_pinion_pos_cmd;
      m_motor_vel_state  = m_motor_vel_cmd;
      return;
    }

    size_t v_pos = feedback.find(",V:", f_pos);
    if (v_pos == std::string::npos) {
      // Invalid format, use command values
      RCLCPP_WARN(m_logger, "PARSE: Feedback message missing ',V:' marker in: '%s'",
                  feedback.c_str());
      m_pinion_pos_state = m_pinion_pos_cmd;
      m_motor_vel_state  = m_motor_vel_cmd;
      return;
    }

    // Extract pinion angle
    std::string pinion_str = feedback.substr(f_pos + 4, v_pos - (f_pos + 4));
    try {
      m_pinion_pos_state = std::stod(pinion_str);
    } catch (const std::exception &e) {
      // Parse error, use command value
      RCLCPP_WARN(m_logger, "PARSE: Failed to parse pinion angle from '%s': %s",
                  pinion_str.c_str(), e.what());
      m_pinion_pos_state = m_pinion_pos_cmd;
    }

    // Extract velocity
    size_t end_pos = feedback.find('\n', v_pos);
    if (end_pos == std::string::npos) { end_pos = feedback.length(); }
    std::string velocity_str = feedback.substr(v_pos + 3, end_pos - (v_pos + 3));
    try {
      m_motor_vel_state = std::stod(velocity_str);
    } catch (const std::exception &e) {
      // Parse error, use command value
      RCLCPP_WARN(m_logger, "PARSE: Failed to parse velocity from '%s': %s",
                  velocity_str.c_str(), e.what());
      m_motor_vel_state = m_motor_vel_cmd;
    }
  }
};

} // namespace igvc_control

// MARK: PLUGIN
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(igvc_control::AckermannArduinoHardware,
                       hardware_interface::SystemInterface)
