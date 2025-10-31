#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "igvc/msg/angle.hpp"
// #include "igvc/msg/diff_state.hpp"

class CmdInterpreter : public rclcpp::Node {
public:
  CmdInterpreter() : Node("cmd_interpreter") {

    // Declare and get parameters
    this->declare_parameter("wheel_base", 0.9144);
    this->declare_parameter("max_steering_angle", 0.785);

    wheel_base_         = this->get_parameter("wheel_base").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    RCLCPP_INFO(this->get_logger(),
                "cmd_interpreter: wheel_base=%.3f m, max_steering_angle=%.3f rad",
                wheel_base_, max_steering_angle_);

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_theta_pub = this->create_publisher<igvc::msg::Angle>("/theta_ideal", 10);

    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) {
          // Update current steering angle from joystick
          double current_steering_angle = 0.0;

          if (msg.angular.z != 0.0) {
            current_steering_angle =
                std::clamp(msg.angular.z, -max_steering_angle_, max_steering_angle_);
            steering_angle_ = current_steering_angle;

            // Publish steering angle when moving
            if (msg.linear.x != 0.0) {
              auto angle_msg  = igvc::msg::Angle();
              angle_msg.theta = current_steering_angle;
              m_theta_pub->publish(angle_msg);
            }
          } else {
            // When no steering input, use the last commanded angle
            current_steering_angle = steering_angle_;
          }

          auto now = this->get_clock()->now();
          if (last_time_.nanoseconds() == 0) {
            last_time_ = now;
            return;
          }

          double dt  = (now - last_time_).seconds();
          last_time_ = now;

          // Skip odometry update if dt is too large (likely from pause/startup)
          if (dt > 1.0) {
            RCLCPP_WARN(this->get_logger(),
                        "Large dt detected: %.3f s, skipping odometry update", dt);
            return;
          }

          const double v = msg.linear.x;

          // Kinematic equations for Ackermann drive
          // Only update if we're actually moving
          if (std::abs(v) > 1e-6) {
            theta_ += current_steering_angle * dt;
            x_ += v * std::cos(theta_) * dt;
            y_ += v * std::sin(theta_) * dt;
          }

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

          // m_tf_broadcaster->sendTransform(t);
        });

    RCLCPP_INFO(this->get_logger(), "cmd_interpreter init()");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_sub;
  rclcpp::Publisher<igvc::msg::Angle>::SharedPtr m_theta_pub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  double wheel_base_;
  double max_steering_angle_;

  double x_{0.0}, y_{0.0}, theta_{0.0};
  double steering_angle_{0.0};
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdInterpreter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}