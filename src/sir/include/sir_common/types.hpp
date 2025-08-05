#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "sir_common/config.hpp"

namespace sir::cfg {

const Config CFG = Config(std::getenv("SIR_CONFIG_PATH"));

// Topics
// ------------------------------------------------------------------------
using TOPIC              = const std::string;
TOPIC IMU_TOPIC          = CFG.get<std::string>("sensor.imu", "topic");
TOPIC GPS_TOPIC          = CFG.get<std::string>("sensor.gps", "topic");
TOPIC POS_ESTIMATE_TOPIC = CFG.get<std::string>("fusion", "topic");
TOPIC GOAL_TOPIC         = "/goal";
TOPIC PATH_TOPIC         = CFG.get<std::string>("pathing", "topic");
TOPIC MAP_TOPIC          = CFG.get<std::string>("mapping", "topic");

const std::chrono::milliseconds
    FUSION_PUBLISH_RATE(CFG.get<int>("fusion", "publish_rate"));

// The world grid has [0,0] being where the robot first localized, and using
// float64 represents it's overall delta movement from there

// the Map is a grid of (u)int's that represent relative space in increments of
// MAP_SIZE / MAP_PRECISION, to translate from the world grid to the map grid
// origin represents the actual localtion on Map[0,0] in the world.
// Every cell in the Map grid can be calcd based on that offsed
struct Map final {
public:
  Map(int8_t d = -1) {
    for (int i = 0; i < NUM_CELLS * NUM_CELLS; ++i) { data.push_back(d); }
  }

  std::vector<int8_t> data;
  double origin_x = 0;
  double origin_y = 0;
  // double origin_theta_z = 0;

  inline void origin(double x, double y) noexcept {
    origin_x = x;
    origin_y = y;
  }

  inline int8_t at(int8_t x, int8_t y) const { return data[y * NUM_CELLS + x]; }

  inline void set(int8_t x, int8_t y, int8_t val) {
    data[y * NUM_CELLS + x] = val;
  }

  inline bool in_bounds(int8_t x, int8_t y) const {
    return x >= 0 && x < NUM_CELLS && y >= 0 && y < NUM_CELLS;
  }

  inline bool w_to_g(double wx, double wy, int8_t &gx, int8_t &gy) const {
    gx = static_cast<int8_t>((wx - origin_x) / MAP_PRECISION);
    gy = static_cast<int8_t>((wy - origin_y) / MAP_PRECISION);
    return in_bounds(gx, gy);
  }

  // Map settings (meters)
  // ------------------------------------------------------------------------
  // length in meters of side of grid
  const int8_t MAP_SIZE = CFG.get<int>("mapping", "size");
  // length in meters of side of cell
  const float MAP_PRECISION = CFG.get<int>("mapping", "precision");
  const size_t NUM_CELLS    = static_cast<size_t>(MAP_SIZE / MAP_PRECISION);
};

} // namespace sir::cfg