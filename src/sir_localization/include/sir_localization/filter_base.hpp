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
  FilterBase(FeedbackMask mask, std::string name)
      : m_mask(mask), m_name(name) {}

  // Optional, implement which is needed for specific filter
  // ------------------------------------------------------------------------
  virtual void process_imu(const sir_msgs::msg::IMUFeedback &imu_msg) {
    (void)imu_msg;
  }
  virtual void process_gps(const sir_msgs::msg::GPSFeedback &gps_msg) {
    (void)gps_msg;
  }

  // Required, Filter must give position estimate
  // ------------------------------------------------------------------------
  virtual sir_msgs::msg::Position_<std::allocator<void>> get_estimate() = 0;

  bool needs(FeedbackType type) {
    return m_mask.test(static_cast<size_t>(type));
  }
  std::string name() { return m_name; }

protected:
  sir_msgs::msg::Position m_current_estimate;

private:
  FeedbackMask m_mask;
  std::string m_name;
};

} // namespace sir::localization