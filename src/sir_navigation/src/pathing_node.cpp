#include <rclcpp/rclcpp.hpp>

#include "sir/msg/map.hpp"
#include "sir/msg/path.hpp"
#include "sir_common/types.hpp"

#include "sir_navigation/astar_pather.hpp"
#include "sir_navigation/dummy_pather.hpp"
#include "sir_navigation/pather_base.hpp"

namespace sir::navigation {

// not sure what used for
const int QOS = 10;

class PathingNode : public rclcpp::Node {
public:
  PathingNode(std::unique_ptr<PatherBase> pather)
      : Node(pather->name() + "_node"), _pather(std::move(pather)) {

    // Create Subscriptions
    // ------------------------------------------------------------------------
    m_goal_sub = this->create_subscription<sir::msg::Position>(
        sir::cfg::GOAL_TOPIC, QOS, [this](const sir::msg::Position &goal) {
          _goal = goal;
          // // publish new path when goal updated
          // m_publisher->publish(
          //     _pather->compute(_position_estimate, _goal, _map));
        });

    m_pos_sub = this->create_subscription<sir::msg::Position>(
        sir::cfg::POS_ESTIMATE_TOPIC, QOS,
        [this](const sir::msg::Position &m) { _position_estimate = m; });

    m_map_sub = this->create_subscription<sir::msg::Map>(
        sir::cfg::MAP_TOPIC, QOS, [this](const sir::msg::Map &map) {
          _map.data = map.data;
          // publish new path when map updated
          m_publisher->publish(_pather->chart(_map, _position_estimate, _goal));
        });

    // Create Publisher
    // ------------------------------------------------------------------------
    m_publisher =
        this->create_publisher<sir::msg::Path>(sir::cfg::PATH_TOPIC, QOS);

    RCLCPP_INFO(this->get_logger(), "Pather Node '%s' initialized",
                _pather->name().c_str());
  }

private:
  rclcpp::Subscription<sir::msg::Position>::SharedPtr m_goal_sub;
  rclcpp::Subscription<sir::msg::Position>::SharedPtr m_pos_sub;
  rclcpp::Subscription<sir::msg::Map>::SharedPtr m_map_sub;

  rclcpp::Publisher<sir::msg::Path>::SharedPtr m_publisher;

  sir::msg::Position _position_estimate;
  sir::msg::Position _goal;
  sir::cfg::Map _map;
  std::unique_ptr<PatherBase> _pather;
};

} // namespace sir::navigation

// Pather Factory
// ------------------------------------------------------------------------
std::unique_ptr<sir::navigation::PatherBase>
create_pather(const std::string &type) {
  using namespace sir::navigation;

  if (type == "dummy") { return std::make_unique<DummyPather>(); }
  if (type == "astar") { return std::make_unique<AStarPather>(); }

  throw std::runtime_error("Unknown pather type: " + type);
}

// Spin up Node
// ------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::string pather_type = "dummy"; // default

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.find("--pather=") == 0) {
      pather_type = arg.substr(std::string("--pather=").size());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Using pather type: %s",
              pather_type.c_str());

  auto pather = create_pather(pather_type);
  auto node = std::make_shared<sir::navigation::PathingNode>(std::move(pather));

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}