#pragma once

#include <rclcpp/rclcpp.hpp>

#include "filter_base.hpp"

namespace sir::localization {

FeedbackMask DUMMY_MASK = FeedbackMask(0);
class DummyFilter final : public FilterBase {
public:
  DummyFilter() : FilterBase(DUMMY_MASK, "dummy_filter") {}

  sir_msgs::msg::Position get_estimate() override {
    auto msg = sir_msgs::msg::Position();

    msg.x = 0;
    msg.y = 0;
    msg.tov = rclcpp::Clock().now();

    return msg;
  }
};

} // namespace sir::localization