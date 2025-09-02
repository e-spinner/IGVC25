#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pather_base.hpp"

namespace igvc::navigation {

class DummyPather final : public PatherBase {
public:
  DummyPather() : PatherBase("dummy_pather") {}

  igvc::msg::Path_<std::allocator<void>>
  chart(igvc::cfg::Map &map, igvc::msg::Position &_start,
        igvc::msg::Position &_goal) override {
    // make clang happy
    (void)_start;
    (void)_goal;
    (void)map;

    auto msg = igvc::msg::Path();
    for (int i = 0; i < 5; ++i) {
      auto pos = igvc::msg::Position();
      pos.x    = i;
      msg.positions.emplace_back(pos);
    }

    msg.time_of_validity = rclcpp::Clock().now();
    return msg;
  }
};

} // namespace igvc::navigation