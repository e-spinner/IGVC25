#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "igvc25/msg/angle.hpp"
// #include "igvc25/msg/diff_state.hpp"

class CmdInterpreter : public rclcpp::Node {
public:
  CmdInterpreter() : Node("cmd_interpreter") {

    // Declare and get parameters
    this->declare_parameter("wheel_base", 0.9144);
    this->declare_parameter("max_steering_angle", 0.785);

    wheel_base_         = this->get_parameter("wheel_base").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    RCLCPP_INFO(
        this->get_logger(),
        "cmd_interpreter: wheel_base=%.3f m, max_steering_angle=%.3f rad",
        wheel_base_, max_steering_angle_);

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_theta_pub =
        this->create_publisher<igvc25::msg::Angle>("/theta_ideal", 10);

    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) {
          float radius =
              (msg.angular.z == 0) ? 0.0 : msg.linear.x / msg.angular.z;

          auto angle = igvc25::msg::Angle();
          angle.theta =
              (radius != 0.0)
                  ? std::clamp(std::atan(wheel_base_ / radius),
                               -max_steering_angle_, max_steering_angle_)
                  : 0.0;

          if (msg.linear.x < 0) { angle.theta = -angle.theta; }

          if (msg.linear.x != 0) { m_theta_pub->publish(angle); }

          // Integrate using Ackermann model
          const double v = msg.linear.x;

          auto now = this->get_clock()->now();
          if (last_time_.nanoseconds() == 0) {
            last_time_ = now;
            return;
          }

          double dt  = (now - last_time_).seconds();
          last_time_ = now;

          const double delta = angle.theta; // steering angle

          // Kinematic equations for Ackermann drive
          double dtheta = (v / wheel_base_) * std::tan(delta) * dt;
          theta_ += dtheta;
          x_ += v * std::cos(theta_) * dt;
          y_ += v * std::sin(theta_) * dt;

          // Broadcast transform
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp            = now;
          t.header.frame_id         = "odom";
          t.child_frame_id          = "base_link";
          t.transform.translation.x = x_;
          t.transform.translation.y = y_;
          t.transform.translation.z = 0.0;

          tf2::Quaternion q;
          q.setRPY(0, 0, theta_);
          t.transform.rotation.x = q.x();
          t.transform.rotation.y = q.y();
          t.transform.rotation.z = q.z();
          t.transform.rotation.w = q.w();

          m_tf_broadcaster->sendTransform(t);
        });

    RCLCPP_INFO(this->get_logger(), "cmd_interpreter init()");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_sub;
  rclcpp::Publisher<igvc25::msg::Angle>::SharedPtr m_theta_pub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  double wheel_base_;
  double max_steering_angle_;

  double x_{0.0}, y_{0.0}, theta_{0.0};
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdInterpreter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}