#include <rclcpp/rclcpp.hpp>

#include "sir_common/types.hpp"
#include "sir_msgs/msg/gps_feedback.hpp"
#include "sir_msgs/msg/imu_feedback.hpp"
#include "sir_msgs/msg/pva.hpp"

#include "sir_localization/dummy_filter.hpp"
#include "sir_localization/filter_base.hpp"

namespace sir::localization {

// not sure what used for
const int QOS = 10;

class FusionNode : public rclcpp::Node {
public:
  FusionNode(std::unique_ptr<FilterBase> filter)
      : Node(filter->name() + "_node"), _filter(std::move(filter)) {

    // Create subscrtiptions, if filter need it
    // ------------------------------------------------------------------------
    if (_filter->needs(FeedbackType::IMU)) {
      m_imu_sub = this->create_subscription<sir_msgs::msg::IMUFeedback>(
          sir::common::IMU_TOPIC, QOS,
          [this](const sir_msgs::msg::IMUFeedback &m) {
            _filter->process_imu(m);
          });
    }

    if (_filter->needs(FeedbackType::GPS)) {
      m_gps_sub = this->create_subscription<sir_msgs::msg::GPSFeedback>(
          sir::common::GPS_TOPIC, QOS,
          [this](const sir_msgs::msg::GPSFeedback &m) {
            _filter->process_gps(m);
          });
    }

    // Create publisher
    // ------------------------------------------------------------------------
    m_publisher = this->create_publisher<sir_msgs::msg::PVA>(
        sir::common::POS_ESTIMATE_TOPIC, QOS);

    RCLCPP_INFO(this->get_logger(), "Fusion Node '%s' initialized",
                _filter->name().c_str());

    // Publish position estimates
    // ------------------------------------------------------------------------
    _timer =
        this->create_wall_timer(sir::common::FUSION_PUBLISH_RATE, [this]() {
          m_publisher->publish(_filter->get_estimate());
        });
  }

private:
  // Callback Functions
  // ------------------------------------------------------------------------
  rclcpp::Subscription<sir_msgs::msg::IMUFeedback>::SharedPtr m_imu_sub;
  rclcpp::Subscription<sir_msgs::msg::GPSFeedback>::SharedPtr m_gps_sub;

  rclcpp::Publisher<sir_msgs::msg::PVA>::SharedPtr m_publisher;

  rclcpp::TimerBase::SharedPtr _timer;
  std::unique_ptr<FilterBase> _filter;
};

} // namespace sir::localization

// Filter Factory
// ------------------------------------------------------------------------
std::unique_ptr<sir::localization::FilterBase>
create_filter(const std::string &type) {
  using namespace sir::localization;

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
      std::make_shared<sir::localization::FusionNode>(std::move(filter));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
