#include <rclcpp/rclcpp.hpp>

#include "sir_common/types.hpp"
#include "sir_msgs/msg/grid_update.hpp"
#include "sir_msgs/msg/path.hpp"
#include "sir_msgs/msg/position.hpp"

#include "sir_navigation/PatherBase.hpp"

namespace sir::navigation {

// not sure what used for
const int QOS = 10;

class PathingNode : public rclcpp::Node {
public:
  PathingNode() : Node("Pathing_Node") {

    // Create Subscriptions
    // ------------------------------------------------------------------------
    m_grid_sub = this->create_subscription<sir_msgs::msg::GridUpdate>(
        sir::common::OBSTACLE_TOPIC, QOS,
        [this](const sir_msgs::msg::GridUpdate &m) {
          _pather->process_update(m);
        });

    m_goal_sub = this->create_subscription<sir_msgs::msg::Position>(
        sir::common::GOAL_TOPIC, QOS, [this](const sir_msgs::msg::Position &m) {
          _pather->set_goal(m);

          // publish new path when goal updated
          m_publisher->publish(_pather->get_path());
        });

    m_pos_sub = this->create_subscription<sir_msgs::msg::Position>(
        sir::common::POS_ESTIMATE_TOPIC, QOS,
        [this](const sir_msgs::msg::Position &m) { _pather->set_pos(m); });

    // Create Publisher
    // ------------------------------------------------------------------------
    m_publisher = this->create_publisher<sir_msgs::msg::Path>(
        sir::common::PATH_TOPIC, QOS);
  }

private:
  rclcpp::Subscription<sir_msgs::msg::GridUpdate>::SharedPtr m_grid_sub;
  rclcpp::Subscription<sir_msgs::msg::Position>::SharedPtr m_goal_sub;
  rclcpp::Subscription<sir_msgs::msg::Position>::SharedPtr m_pos_sub;

  rclcpp::Publisher<sir_msgs::msg::Path>::SharedPtr m_publisher;

  std::unique_ptr<PatherBase> _pather;
};

} // namespace sir::navigation