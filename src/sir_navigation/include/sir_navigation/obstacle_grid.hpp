#pragma once

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "sir_common/types.hpp"
#include "sir_msgs/msg/grid_update.hpp"

namespace sir::navigation {

const size_t GRID_SIZE =
    sir::common::MAX_OBSTACLE_DISTANCE / sir::common::OBSTACLE_PRECISION;

enum ROLLOVER : uint8_t { POSITIVE_X, NEGATIVE_X, POSITIVE_Y, NEGATIVE_Y };

class ObstacleGrid final {
public:
  enum CellState : uint8_t { FREE = 0, OCCUPIED = 1, UNKNOWN = 255 };

  ObstacleGrid() = default;

  void apply_update(const sir_msgs::msg::GridUpdate &msg) {
    std::lock_guard lock(m_mutex);
    for (const auto u : msg.updates) {
      m_data[u.x][u.y] = u.occupied ? OCCUPIED : FREE;
    }
  }

  // Returns the state of a specific cell (or UNKNOWN if out-of-bounds)
  CellState get_cell(size_t x, size_t y) const {
    std::lock_guard lock(m_mutex);
    if (x >= GRID_SIZE || y >= GRID_SIZE) return UNKNOWN;
    return m_data[x][y];
  }

  // Get a full snapshot of the grid (copy)
  std::array<std::array<CellState, GRID_SIZE>, GRID_SIZE> get_snapshot() const {
    std::lock_guard lock(m_mutex);
    return m_data;
  }

  void grid_rollover(ROLLOVER rollover) {
    std::lock_guard lock(m_mutex);

    switch (rollover) {
    case ROLLOVER::POSITIVE_X:
      for (size_t y = 0; y < GRID_SIZE; ++y) {
        for (size_t x = GRID_SIZE - 1; x > 0; --x) {
          m_data[x][y] = m_data[x - 1][y];
        }
        m_data[0][y] = UNKNOWN;
      }
      break;
    case NEGATIVE_X:
      for (size_t y = 0; y < GRID_SIZE; ++y) {
        for (size_t x = 0; x < GRID_SIZE - 1; ++x) {
          m_data[x][y] = m_data[x + 1][y];
        }
        m_data[GRID_SIZE - 1][y] = UNKNOWN;
      }
      break;

    case POSITIVE_Y:
      for (size_t x = 0; x < GRID_SIZE; ++x) {
        for (size_t y = GRID_SIZE - 1; y > 0; --y) {
          m_data[x][y] = m_data[x][y - 1];
        }
        m_data[x][0] = UNKNOWN;
      }
      break;

    case NEGATIVE_Y:
      for (size_t x = 0; x < GRID_SIZE; ++x) {
        for (size_t y = 0; y < GRID_SIZE - 1; ++y) {
          m_data[x][y] = m_data[x][y + 1];
        }
        m_data[x][GRID_SIZE - 1] = UNKNOWN;
      }
      break;
    }
  }

  // for consumers
  static constexpr size_t size() { return GRID_SIZE; }

private:
  std::array<std::array<CellState, GRID_SIZE>, GRID_SIZE> m_data{UNKNOWN};

  mutable std::mutex m_mutex;
};

} // namespace sir::navigation