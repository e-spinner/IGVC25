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
TOPIC PATH_TOPIC         = "/path";

constexpr std::chrono::milliseconds FUSION_PUBLISH_RATE(100); // 10 hz

} // namespace sir::common