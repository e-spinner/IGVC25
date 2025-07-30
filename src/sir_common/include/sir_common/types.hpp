#include <chrono>
#include <string>

namespace sir::common {

const std::string IMU_TOPIC = "/imu";
const std::string GPS_TOPIC = "/gps";
const std::string POS_ESTIMATE_TOPIC = "/position_estimate";

constexpr std::chrono::milliseconds FUSION_PUBLISH_RATE(100); // 10 hz

} // namespace sir::common