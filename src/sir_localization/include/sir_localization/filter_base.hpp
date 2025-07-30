#pragma once

#include <bitset>

#include "sir_msgs/msg/gps_feedback.hpp"
#include "sir_msgs/msg/imu_feedback.hpp"
#include "sir_msgs/msg/position.hpp"

namespace sir::localization {

// https://www.ri.cmu.edu/pub_files/2016/5/awerries_written_qualifier.pdf
// https://ahrs.readthedocs.io/en/latest/filters/ekf.html
// https://pointonenav.com/news/loose-vs-tight-coupling-gnss/

enum class FeedbackType : u_int8_t {
  IMU,
  GPS,
  // PSUEDO_RANGE,
  // AHRS,
  // ODOM,
  // CMD_VEL,

  M_COUNT
};

using FeedbackMask = std::bitset<static_cast<size_t>(FeedbackType::M_COUNT)>;

// filters will implement FilterBase, selecting Sensos via FeedbackMask

class FilterBase {
public:
  FilterBase() = default;

  virtual void process_imu(const sir_msgs::msg::IMUFeedback &imu_msg) = 0;
  virtual void process_gps(const sir_msgs::msg::GPSFeedback &gps_msg) = 0;

  virtual sir_msgs::msg::Position_<std::allocator<void>> get_estimate() = 0;

private:
  sir_msgs::msg::Position m_current_estimate;
};

} // namespace sir::localization