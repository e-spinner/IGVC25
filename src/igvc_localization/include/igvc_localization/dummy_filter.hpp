#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "filter_base.hpp"

namespace igvc::localization {

const FeedbackMask DUMMY_MASK = FeedbackMask(0);
class DummyFilter final : public FilterBase {
public:
  DummyFilter() : FilterBase(DUMMY_MASK, "dummy_filter") {}

  nav_msgs::msg::Odometry get_estimate() override {
    nav_msgs::msg::Odometry msg;

    // Fill header (stamp + frame_id)
    msg.header.stamp    = rclcpp::Clock().now();
    msg.header.frame_id = "odom"; // or "map"

    // Position
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // Orientation (identity quaternion)
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    // Twist (velocities) — zero for dummy
    msg.twist.twist.linear.x  = 0.0;
    msg.twist.twist.linear.y  = 0.0;
    msg.twist.twist.linear.z  = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    return msg;
  }
};

} // namespace igvc::localization