#pragma once

#include "sir_msgs/msg/gps_feedback.hpp"
#include "sir_msgs/msg/imu_feedback.hpp"
#include "sir_msgs/msg/position.hpp"

namespace sir::localization {

// https://www.ri.cmu.edu/pub_files/2016/5/awerries_written_qualifier.pdf

// need to take in GNSS / IMU / CMD_VEL / ODOM

class FilterEngine final {
public:
  FilterEngine() = default;

  void process_imu(const sir_msgs::msg::IMUFeedback &imu_msg) {}
  void process_gps(const sir_msgs::msg::GPSFeedback &gps_msg) {}

  sir_msgs::msg::Position_<std::allocator<void>> get_estimate() {
    auto estimate = sir_msgs::msg::Position();

    return estimate;
  }

private:
  sir_msgs::msg::Position m_current_estimate;
};

} // namespace sir::localization