#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <string>

#include "igvc25/msg/root_instruction.hpp"

const int QOS                = 10;
const std::string ROOT_TOPIC = "/root_instr";

class TruthGen final : public rclcpp::Node {
public:
  TruthGen() : Node("ground_truth_generator") {

    publisher =
        this->create_publisher<nav_msgs::msg::Odometry>("/truth_pva", 50);

    subscriber = this->create_subscription<igvc25::msg::RootInstruction>(
        ROOT_TOPIC, QOS,
        [this](const igvc25::msg::RootInstruction::SharedPtr msg) {
          // immediatly start executing new instuction
          current_instr   = *msg;
          instr_time      = 0.0;
          has_instruction = true;

          RCLCPP_INFO(this->get_logger(),
                      "Received instruction: L=%.2f, R=%.2f, dur=%.2f",
                      current_instr.left_speed, current_instr.right_speed,
                      current_instr.duration);
        });

    timer = this->create_wall_timer(std::chrono::milliseconds(20),
                                    std::bind(&TruthGen::update, this));
  }

private:
  void update() {
    if (!has_instruction) return;

    const double WB{10.2};
    v     = (current_instr.left_speed + current_instr.right_speed) / 2.0;
    omega = (current_instr.right_speed - current_instr.left_speed) / WB;

    // kinematics
    x += (v / 100.0) * cos(theta_z) * delta_time;
    y += (v / 100.0) * sin(theta_z) * delta_time;
    theta_z += omega * delta_time;

    // publish odometry
    auto msg = nav_msgs::msg::Odometry();

    msg.header.stamp    = this->now();
    msg.header.frame_id = "pva";
    msg.child_frame_id  = "base";

    msg.pose.pose.position.x    = x;
    msg.pose.pose.position.y    = y;
    msg.pose.pose.orientation.z = sin(theta_z / 2.0);
    msg.pose.pose.orientation.w = cos(theta_z / 2.0);
    msg.twist.twist.linear.x    = v / 100.0;
    msg.twist.twist.angular.z   = omega;
    publisher->publish(msg);

    // update timer
    instr_time += delta_time;
    if (instr_time >= current_instr.duration) { has_instruction = false; }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
  rclcpp::Subscription<igvc25::msg::RootInstruction>::SharedPtr subscriber;
  rclcpp::TimerBase::SharedPtr timer;

  double x{0.0}, y{0.0}, theta_z{0.0};
  double v{0.0}, omega{0.0};
  double instr_time{0.0}, delta_time{0.02};
  bool has_instruction{false};

  igvc25::msg::RootInstruction current_instr;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TruthGen>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}