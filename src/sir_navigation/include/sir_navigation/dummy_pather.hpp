#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pather_base.hpp"

namespace sir::navigation {

class DummyPather final : public PatherBase {
public:
  DummyPather() : PatherBase("dummy_pather") {}

  sir::msg::Path_<std::allocator<void>> compute(sir::msg::Position start,
                                                sir::msg::Position goal,
                                                OccupancyMap map) override {
    auto msg = sir::msg::Path();
    for (int i = 0; i < 5; ++i) {
      auto pos = sir::msg::Position();
      pos.x    = i;
      msg.positions.emplace_back(pos);
    }

    msg.time_of_validity = rclcpp::Clock().now();
    return msg;
  }
};

} // namespace sir::navigation