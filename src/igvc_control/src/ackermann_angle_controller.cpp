#include <controller_interface/controller_interface.hpp>

namespace ackermann_angle_controller {

class AckermannAngleController final
    : public controller_interface::ControllerInterface {
public:
  AckermannAngleController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

private:
};

int main(int argc, char **argv) {}