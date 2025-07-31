#pragma once

#include <rclcpp/rclcpp.hpp>

#include "filter_base.hpp"

namespace sir::localization {

FeedbackMask DUMMY_MASK = FeedbackMask(0);
class DummyFilter final : public FilterBase {
public:
  DummyFilter() : FilterBase(DUMMY_MASK, "dummy_filter") {}

  sir_msgs::msg::PVA get_estimate() override {
    auto msg = sir_msgs::msg::PVA();

    msg.position.x = 0;
    msg.position.y = 0;
    msg.time_of_validity = rclcpp::Clock().now();

    return msg;
  }
};

} // namespace sir::localization