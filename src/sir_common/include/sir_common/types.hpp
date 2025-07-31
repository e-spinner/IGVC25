#include <chrono>
#include <string>

namespace sir::common {

// Topics
// ------------------------------------------------------------------------
using TOPIC              = const std::string;
TOPIC IMU_TOPIC          = "/imu";
TOPIC GPS_TOPIC          = "/gps";
TOPIC POS_ESTIMATE_TOPIC = "/position_estimate";
TOPIC GOAL_TOPIC         = "/goal";
TOPIC OBSTACLE_TOPIC     = "/grid_update";
TOPIC PATH_TOPIC         = "/path";

constexpr std::chrono::milliseconds FUSION_PUBLISH_RATE(100); // 10 hz

// Obstacle detection settings (meters)
// ------------------------------------------------------------------------
const float MAX_OBSTACLE_DISTANCE = 10;
const float OBSTACLE_PRECISION    = 0.25;
} // namespace sir::common