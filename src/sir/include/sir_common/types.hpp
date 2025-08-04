#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace sir::cfg {

// Topics
// ------------------------------------------------------------------------
using TOPIC              = const std::string;
TOPIC IMU_TOPIC          = "/imu";
TOPIC GPS_TOPIC          = "/gps";
TOPIC POS_ESTIMATE_TOPIC = "/position_estimate";
TOPIC GOAL_TOPIC         = "/goal";
TOPIC PATH_TOPIC         = "/path";
TOPIC MAP_TOPIC          = "/map";

constexpr std::chrono::milliseconds FUSION_PUBLISH_RATE(100); // 10 hz

// Map settings (meters)
// ------------------------------------------------------------------------
const size_t MAP_SIZE     = 10;
const float MAP_PRECISION = 1;

struct Map final {
public:
  Map(int8_t d = -1) {
    for (int i = 0; i < MAP_SIZE * MAP_SIZE; ++i) { data.push_back(d); }
  }

  std::vector<int8_t> data;

  inline int8_t at(int8_t x, int8_t y) const { return data[y * MAP_SIZE + x]; }

  inline void set(int8_t x, int8_t y, int8_t val) {
    data[y * MAP_SIZE + x] = val;
  }

  inline bool in_bounds(int8_t x, int8_t y) const {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
  }
};

} // namespace sir::cfg