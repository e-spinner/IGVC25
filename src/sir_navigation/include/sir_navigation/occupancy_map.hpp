#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "sir/msg/map.hpp"
#include "sir_common/types.hpp"

namespace sir::navigation {

class OccupancyMap final {
public:
  enum CellState : int8_t { FREE = 0, OCCUPIED = 1, UNKNOWN = -1 };

  OccupancyMap() = default;

  void update_map(const sir::msg::Map &map) {
    if (map.data.size() != sir::common::MAP_HEIGHT * sir::common::MAP_WIDTH)
      throw std::runtime_error("Map data size mismatch.");

    m_origin = map.origin;
    m_data   = map.data;
  }

  // Convert world → grid (map-relative) indices
  inline bool world_to_grid(float wx, float wy, int &gx, int &gy) const {
    gx = static_cast<int>((wx - m_origin.x) / sir::common::MAP_PRECISION);
    gy = static_cast<int>((wy - m_origin.y) / sir::common::MAP_PRECISION);
    return gx >= 0 && gx < static_cast<int>(sir::common::MAP_WIDTH) &&
           gy >= 0 && gy < static_cast<int>(sir::common::MAP_HEIGHT);
  }

  // Convert grid (map-relative) → world coordinates
  inline void grid_to_world(int gx, int gy, float &wx, float &wy) const {
    wx = m_origin.x + (gx + 0.5f) * sir::common::MAP_PRECISION;
    wy = m_origin.y + (gy + 0.5f) * sir::common::MAP_PRECISION;
  }

  // Read from the data grid
  inline CellState at(int gx, int gy) const {
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(sir::common::MAP_WIDTH) ||
        gy >= static_cast<int>(sir::common::MAP_HEIGHT))
      return UNKNOWN;
    return static_cast<CellState>(m_data[gy * sir::common::MAP_WIDTH + gx]);
  }

  size_t width() const { return sir::common::MAP_WIDTH; }
  size_t height() const { return sir::common::MAP_HEIGHT; }

private:
  sir::msg::Position m_origin;
  std::vector<int8_t> m_data;
};

} // namespace sir::navigation
