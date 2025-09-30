#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "igvc25/dummy_filter.hpp"
#include "igvc25/filter_base.hpp"

namespace igvc::localization {

const int QOS                  = 10;
const std::string IMU_TOPIC    = "/vision/imu/data_raw";
const std::string GPS_TOPIC    = "/vision/fix";
const std::string FUSION_TOPIC = "/odom/estimate";
constexpr std::chrono::milliseconds FUSION_PUBLISH_RATE(100); // 10 hz

class FusionNode : public rclcpp::Node {
public:
  FusionNode(std::unique_ptr<FilterBase> filter)
      : Node(filter->name() + "_node"), _filter(std::move(filter)) {

    // Create subscrtiptions, if filter need it
    // ------------------------------------------------------------------------
    if (_filter->needs(FeedbackType::IMU)) {
      m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
          IMU_TOPIC, QOS, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            _filter->process_imu(*msg);
          });
    }

    if (_filter->needs(FeedbackType::GPS)) {
      m_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          GPS_TOPIC, QOS, [this](const sensor_msgs::msg::NavSatFix &msg) {
            _filter->process_gps(msg);
          });
    }

    // Create publisher
    // ------------------------------------------------------------------------
    m_publisher =
        this->create_publisher<nav_msgs::msg::Odometry>(FUSION_TOPIC, QOS);

    RCLCPP_INFO(this->get_logger(), "Fusion Node '%s' initialized",
                _filter->name().c_str());

    // Publish position estimates
    // ------------------------------------------------------------------------
    _timer = this->create_wall_timer(FUSION_PUBLISH_RATE, [this]() {
      m_publisher->publish(_filter->get_estimate());
    });
  }

private:
  // Callback Functions
  // ------------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;

  rclcpp::TimerBase::SharedPtr _timer;
  std::unique_ptr<FilterBase> _filter;
};

} // namespace igvc::localization

// Filter Factory
// ------------------------------------------------------------------------
std::unique_ptr<igvc::localization::FilterBase>
create_filter(const std::string &type) {
  using namespace igvc::localization;

  if (type == "dummy") { return std::make_unique<DummyFilter>(); }

  throw std::runtime_error("Unknown filter type: " + type);
}

// Spin up node
// ------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::string filter_type = "dummy"; // TODO: Get from param/env/arg

  auto filter = create_filter(filter_type);
  auto node =
      std::make_shared<igvc::localization::FusionNode>(std::move(filter));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
