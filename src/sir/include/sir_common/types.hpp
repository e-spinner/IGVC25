#pragma once

#include <chrono>
#include <string>

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
const float MAP_WIDTH     = 10;
const float MAP_HEIGHT    = 10;
const float MAP_PRECISION = 0.25;
} // namespace sir::cfg