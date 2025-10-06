#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>

#include "igvc25/msg/root_instruction.hpp"

// this node listens for a RootInstruction msg from root_controller.py and
// generates a truth odometry for use with testing different pos-estimation algs
// later the goal is to collect data by straping the sensors to a root rt1 and
// control it with the same exact instruction as this recieves, and put this
// output + collect sensor data into a rosbag to use as a test case with our
// specific sensors and computer

const int QOS                = 10;
const std::string ROOT_TOPIC = "/root_instr";

class TruthGen final : public rclcpp::Node {
public:
  TruthGen() : Node("ground_truth_generator") {

    odom_publisher =
        this->create_publisher<nav_msgs::msg::Odometry>("/truth_pva", 50);

    joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 50);

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
    msg.header.frame_id = "odom";
    msg.child_frame_id  = "base_link";

    msg.pose.pose.position.x    = x;
    msg.pose.pose.position.y    = y;
    msg.pose.pose.orientation.z = sin(theta_z / 2.0);
    msg.pose.pose.orientation.w = cos(theta_z / 2.0);
    msg.twist.twist.linear.x    = v / 100.0;
    msg.twist.twist.angular.z   = omega;
    odom_publisher->publish(msg);

    // joint states
    l_w_angle += ((current_instr.left_speed / 100.0) / 0.0159) * delta_time;
    r_w_angle += ((current_instr.right_speed / 100.0) / 0.0159) * delta_time;

    l_w_angle = fmod(l_w_angle, 2 * M_PI);
    r_w_angle = fmod(r_w_angle, 2 * M_PI);

    auto j_msg         = sensor_msgs::msg::JointState();
    j_msg.header.stamp = this->now();
    j_msg.name         = {"left_wheel_joint", "right_wheel_joint"};
    j_msg.position     = {l_w_angle, r_w_angle};
    j_msg.velocity     = {current_instr.left_speed, current_instr.right_speed};
    joint_publisher->publish(j_msg);

    // odom transform
    auto tf            = geometry_msgs::msg::TransformStamped();
    tf.header.stamp    = this->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_z);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(tf);

    // update timer
    instr_time += delta_time;
    if (instr_time >= current_instr.duration) { has_instruction = false; }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<igvc25::msg::RootInstruction>::SharedPtr subscriber;
  rclcpp::TimerBase::SharedPtr timer;

  double x{0.0}, y{0.0}, theta_z{0.0};
  double v{0.0}, omega{0.0};
  double l_w_angle{0.0}, r_w_angle{0.0};
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