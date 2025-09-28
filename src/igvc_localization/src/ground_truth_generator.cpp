#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <vector>

#include "igvc_localization/instructions.hpp"

class TruthGen : public rclcpp::Node {
public:
  TruthGen() : Node("ground_truth_generator") {

    publisher =
        this->create_publisher<nav_msgs::msg::Odometry>("/truth_pva", 50);

    timer = this->create_wall_timer(std::chrono::milliseconds(20),
                                    std::bind(&TruthGen::update, this));
  }

  void set_instructions(const std::vector<Instruction> instrs) {
    instructions = instrs;
  }

private:
  void update() {
    if (current_instr >= instructions.size()) return;

    Instruction &instr = instructions[current_instr];

    // State
    if (instr.type == "MOVE") {
      v     = instr.speed;
      omega = 0.0;
    } else if (instr.type == "TURN") {
      v     = 0.0;
      omega = instr.speed;
    }

    // kinematics
    x += v * cos(theta_z) * delta_time;
    y += v * sin(theta_z) * delta_time;
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
    msg.twist.twist.linear.x    = v;
    msg.twist.twist.angular.z   = omega;
    publisher->publish(msg);

    // update timer
    instr_time += delta_time;
    if (instr_time >= instr.duration) {
      current_instr++;
      instr_time = 0.0;
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  double x{0.0}, y{0.0}, theta_z{0.0};
  double v{0.0}, omega{0.0};
  size_t current_instr{0};
  double instr_time{0.0}, delta_time{0.02};
  std::vector<Instruction> instructions;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::string choice = "square"; // default
  if (argc > 1) { choice = argv[1]; }

  auto node = std::make_shared<TruthGen>();

  if (choice == "square")
    node->set_instructions(square_path());
  else {
    RCLCPP_WARN(node->get_logger(),
                "Unknown instruction set '%s', using default square path.",
                choice.c_str());
    node->set_instructions(square_path());
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}