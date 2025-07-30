#include <rclcpp/rclcpp.hpp>

#include "sir_common/types.hpp"
#include "sir_msgs/msg/gps_feedback.hpp"
#include "sir_msgs/msg/imu_feedback.hpp"
#include "sir_msgs/msg/position.hpp"

#include "sir_localization/filter_engine.hpp"

using std::placeholders::_1;

namespace sir::localization {

// not sure what used for
const int QOS = 10;

class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node") {

    // Create subscrtiptions
    // ------------------------------------------------------------------------
    m_imu_sub = this->create_subscription<sir_msgs::msg::IMUFeedback>(
        sir::common::IMU_TOPIC, QOS,
        std::bind(&FusionNode::imu_callback, this, _1));

    m_gps_sub = this->create_subscription<sir_msgs::msg::GPSFeedback>(
        sir::common::GPS_TOPIC, QOS,
        std::bind(&FusionNode::gps_callback, this, _1));

    // Create publisher
    // ------------------------------------------------------------------------
    m_publisher = this->create_publisher<sir_msgs::msg::Position>(
        sir::common::POS_ESTIMATE_TOPIC, QOS);

    RCLCPP_INFO(this->get_logger(), "Fusion Node Intialized");

    // Publish position estimates
    // ------------------------------------------------------------------------
    _timer =
        this->create_wall_timer(sir::common::FUSION_PUBLISH_RATE,
                                std::bind(&FusionNode::pub_callback, this));
  }

private:
  // Callback Functions
  // ------------------------------------------------------------------------
  rclcpp::Subscription<sir_msgs::msg::IMUFeedback>::SharedPtr m_imu_sub;
  void imu_callback(const sir_msgs::msg::IMUFeedback &msg) {
    _filter.process_imu(msg);
  }

  rclcpp::Subscription<sir_msgs::msg::GPSFeedback>::SharedPtr m_gps_sub;
  void gps_callback(const sir_msgs::msg::GPSFeedback &msg) {
    _filter.process_gps(msg);
  }

  rclcpp::Publisher<sir_msgs::msg::Position>::SharedPtr m_publisher;
  void pub_callback() { m_publisher->publish(_filter.get_estimate()); }

  rclcpp::TimerBase::SharedPtr _timer;
  sir::localization::FilterEngine _filter;
};

} // namespace sir::localization

// Spin up node
// ------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sir::localization::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
