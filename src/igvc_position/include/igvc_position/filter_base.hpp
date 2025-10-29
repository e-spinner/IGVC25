#pragma once

#include <bitset>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace igvc::localization {

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

  m_COUNT
};

using FeedbackMask = std::bitset<static_cast<size_t>(FeedbackType::m_COUNT)>;

// filters will implement FilterBase, selecting Sensors via FeedbackMask

class FilterBase {
public:
  FilterBase(FeedbackMask mask, std::string name)
      : m_mask(mask), m_name(name) {}

  // Optional, implement which is needed for specific filter
  // ------------------------------------------------------------------------
  virtual void process_imu(const sensor_msgs::msg::Imu &imu_msg) {
    (void)imu_msg;
  }
  virtual void process_gps(const sensor_msgs::msg::NavSatFix &gps_msg) {
    (void)gps_msg;
  }

  // Required, Filter must give position estimate
  // ------------------------------------------------------------------------
  virtual nav_msgs::msg::Odometry_<std::allocator<void>> get_estimate() = 0;

  bool needs(FeedbackType type) {
    return m_mask.test(static_cast<size_t>(type));
  }

  std::string name() { return m_name; }

  // protected:
  nav_msgs::msg::Odometry m_current_estimate;

private:
  FeedbackMask m_mask;
  std::string m_name;
};

} // namespace igvc::localization