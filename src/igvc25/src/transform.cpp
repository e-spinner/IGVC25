#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hack13/msg/ack_state.hpp"

class AckJointPublisher : public rclcpp::Node {
public:
  AckJointPublisher() : Node("ack_joint_publisher") {
    joint_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    ack_state_sub_ = this->create_subscription<hack13::msg::AckState>(
        "/ack_state", 10,
        [this](const hack13::msg::AckState &msg) { publish_joint_state(msg); });

    RCLCPP_INFO(this->get_logger(), "ack_joint_publisher initialized");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<hack13::msg::AckState>::SharedPtr ack_state_sub_;

  void publish_joint_state(const hack13::msg::AckState &msg) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();

    // List of joint names must match URDF
    js.name = {"pin_one",   "pin_two",  "rack_joint",
               "pin_three", "pin_four", "pinion_joint"};

    // Corresponding positions from AckState message
    js.position = {msg.theta_2, msg.theta_3,
                   msg.d, // rack displacement (assuming translational)
                   msg.theta_5, msg.theta_4, msg.theta_pin};

    // Velocities/efforts optional
    js.velocity.resize(js.name.size(), 0.0);
    js.effort.resize(js.name.size(), 0.0);

    joint_pub_->publish(js);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckJointPublisher>());
  rclcpp::shutdown();
  return 0;
}
