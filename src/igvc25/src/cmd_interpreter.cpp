#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "hack13/msg/angle.hpp"
// #include "hack13/msg/diff_state.hpp"

class CmdInterpreter : public rclcpp::Node {
public:
  CmdInterpreter() : Node("cmd_interpreter") {

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_theta_pub = this->create_publisher<hack13::msg::Angle>("/theta_ideal", 10);
    // m_diff_pub  = this->create_publisher<hack13::msg::DiffState>("/diff_state",
    // 10);

    m_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) {
          float radius = (msg.angular.z == 0) ? 0.0 : msg.linear.x / msg.angular.z;

          auto angle  = hack13::msg::Angle();
          angle.theta = (radius != 0.0) ? std::clamp(std::atan(WHEEL_BASE / radius),
                                                     float(-0.95), float(0.95))
                                        : 0.0;

          if (msg.linear.x < 0) { angle.theta = -angle.theta; }

          if (msg.linear.x != 0) { m_theta_pub->publish(angle); }

          // auto diff = hack13::msg::DiffState();

          // diff.v_left = (angle.theta < 0)
          //                   ? msg.angular.z * (radius - (WHEEL_BASE / 2))
          //                   : msg.angular.z * (radius + (WHEEL_BASE / 2));

          // diff.v_right = (angle.theta < 0)
          //                    ? msg.angular.z * (radius + (WHEEL_BASE / 2))
          //                    : msg.angular.z * (radius - (WHEEL_BASE / 2));

          // m_diff_pub->publish(diff);

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
          double dtheta = (v / WHEEL_BASE) * std::tan(delta) * dt;
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
  rclcpp::Publisher<hack13::msg::Angle>::SharedPtr m_theta_pub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  // rclcpp::Publisher<hack13::msg::DiffState>::SharedPtr m_diff_pub;

  constexpr static const float WHEEL_BASE{0.7};

  double x_, y_, theta_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdInterpreter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}