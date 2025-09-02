#include <rclcpp/rclcpp.hpp>

#include "igvc/msg/map.hpp"
#include "igvc/msg/path.hpp"
#include "igvc_common/types.hpp"

const int QOS = 10;

const std::vector<int8_t> MAP = {
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, //
    0, 1, 0, 0, 0, 1, 0, 0, 0, 0, //
    0, 1, 0, 1, 0, 1, 0, 0, 0, 0, //
    0, 1, 0, 1, 0, 1, 1, 1, 0, 1, //
    0, 0, 0, 1, 0, 0, 0, 1, 0, 0, //
    0, 0, 0, 1, 0, 1, 1, 0, 1, 1, //
    0, 0, 1, 1, 1, 1, 0, 1, 0, 0, //
    0, 1, 0, 1, 0, 1, 0, 0, 1, 0, //
    0, 0, 1, 0, 1, 0, 0, 0, 1, 0, //
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0  //
};

class MapNode : public rclcpp::Node {
public:
  MapNode() : Node("map_tester") {
    m_path_sub = this->create_subscription<igvc::msg::Path>(
        igvc::cfg::PATH_TOPIC, QOS, [this](const igvc::msg::Path &path) {
          RCLCPP_INFO(this->get_logger(),
                      "Received path with %zu poses:", path.positions.size());
          igvc::msg::Position start, goal;
          start.x = 0;
          start.y = 0;
          goal.x  = 9;
          goal.y  = 9;

          print_map_with_path(MAP, 10, 10, path.positions, start, goal);
        });

    m_pos_pub = this->create_publisher<igvc::msg::Position>(
        igvc::cfg::POS_ESTIMATE_TOPIC, QOS);

    auto pos = igvc::msg::Position();
    pos.x    = 0;
    pos.y    = 0;
    m_pos_pub->publish(pos);

    m_goal_pub =
        this->create_publisher<igvc::msg::Position>(igvc::cfg::GOAL_TOPIC, QOS);

    auto goal = igvc::msg::Position();
    goal.x    = 9;
    goal.y    = 9;
    m_goal_pub->publish(goal);

    m_publisher =
        this->create_publisher<igvc::msg::Map>(igvc::cfg::MAP_TOPIC, QOS);
    auto map             = igvc::msg::Map();
    auto origin          = igvc::msg::Position();
    origin.x             = 0;
    origin.y             = 0;
    map.origin           = origin;
    map.data             = MAP;
    map.time_of_validity = rclcpp::Clock().now();
    m_publisher->publish(map);
  }

private:
  rclcpp::Subscription<igvc::msg::Path>::SharedPtr m_path_sub;

  rclcpp::Publisher<igvc::msg::Position>::SharedPtr m_pos_pub;
  rclcpp::Publisher<igvc::msg::Position>::SharedPtr m_goal_pub;
  rclcpp::Publisher<igvc::msg::Map>::SharedPtr m_publisher;

  void print_map_with_path(const std::vector<int8_t> &map, int width,
                           int height,
                           const std::vector<igvc::msg::Position> &path,
                           const igvc::msg::Position &start,
                           const igvc::msg::Position &goal) {
    std::vector<std::string> grid(height, std::string(width, '.'));

    // Mark obstacles
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (map[y * width + x] == 1) { grid[y][x] = ' '; }
      }
    }

    // Mark path
    for (const auto &p : path) {
      int x = static_cast<int>(p.x);
      int y = static_cast<int>(p.y);
      if (x >= 0 && x < width && y >= 0 && y < height) grid[y][x] = '*';
    }

    // Mark start and goal
    int sx = static_cast<int>(start.x);
    int sy = static_cast<int>(start.y);
    int gx = static_cast<int>(goal.x);
    int gy = static_cast<int>(goal.y);

    if (sx >= 0 && sx < width && sy >= 0 && sy < height) grid[sy][sx] = 'S';
    if (gx >= 0 && gx < width && gy >= 0 && gy < height) grid[gy][gx] = 'G';

    // Print top-down
    std::cout << "\nMap:\n";
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) { std::cout << grid[y][x] << ' '; }
      std::cout << "\n";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}