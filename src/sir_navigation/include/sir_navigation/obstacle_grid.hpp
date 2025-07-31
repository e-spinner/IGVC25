#pragma once

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "sir_msgs/msg/grid_update.hpp"

namespace sir::navigation {

class ObstacleGrid final {
public:
  enum CellState : uint8_t {
    FREE     = 0,
    OCCUPIED = 1,
    UNKNOWN  = 255,
  };

  ObstacleGrid(float cell_size, float range);

  void apply_update(const sir_msgs::msg::GridUpdate &msg) {
    std::lock_guard lock(m_mutex);
    for (const auto u : msg.updates) {}
  }

private:
  mutable std::mutex m_mutex;
};

} // namespace sir::navigation