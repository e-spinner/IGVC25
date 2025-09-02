#pragma once

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "igvc_common/types.hpp"

#include "pather_base.hpp"

namespace igvc::navigation {

// https://github.com/JDSherbert/A-Star-Pathfinding/blob/main/Pathfinder.cpp
// https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
class AStarPather final : public PatherBase {
public:
  AStarPather() : PatherBase("astar_pather") {}

  igvc::msg::Path_<std::allocator<void>>
  chart(igvc::cfg::Map &map, igvc::msg::Position &_start,
        igvc::msg::Position &_goal) override {

    const float SIDE     = map.MAP_PRECISION;
    const float DIAGONAL = std::sqrt(2 * SIDE);

    int8_t tmp_x, tmp_y;
    map.w_to_g(_start.x, _start.y, tmp_x, tmp_y);
    Node start{tmp_x, tmp_y};
    map.w_to_g(_goal.x, _goal.y, tmp_x, tmp_y);
    Node goal{tmp_x, tmp_y};

    std::priority_queue<Node, std::vector<Node>, compare> open_set;
    // stores g score of each visited, -1 = not visited
    igvc::cfg::Map closed_set(-1);
    // stores parent of each Node
    std::vector<std::vector<std::pair<int8_t, int8_t>>> came_from(
        map.NUM_CELLS, std::vector<std::pair<int8_t, int8_t>>(map.NUM_CELLS));

    closed_set.set(_start.x, start.y, 0);
    open_set.push(start);

    bool path_found = false;

    while (!open_set.empty()) {
      Node current = open_set.top();
      open_set.pop();

      if (current == goal) {
        path_found = true;
        break;
      }

      // explore neighbors
      for (auto &[dx, dy] : KDirections) {
        int8_t x = current.x + dx;
        int8_t y = current.y + dy;
        if (map.in_bounds(x, y)) {
          // if neighbor is occupied or already closed
          if (map.at(x, y) != 1) {

            float tentative_g =
                current.g + dx == 0 || dy == 0 ? SIDE : DIAGONAL;

            // check if neighbor has been visited, or if it has, if this is
            // better
            if (closed_set.at(x, y) == -1 ||
                tentative_g < closed_set.at(x, y)) {
              Node neighbor(x, y);
              neighbor.g      = tentative_g;
              neighbor.h      = heuristic(neighbor, goal, SIDE, DIAGONAL);
              came_from[x][y] = {current.x, current.y};
              open_set.push(neighbor);
              closed_set.set(x, y, tentative_g);
            }
          }
        }
      }
    }

    // reconstruct path
    auto path             = igvc::msg::Path();
    path.time_of_validity = rclcpp::Clock().now();
    auto origin           = igvc::msg::Position();
    origin.x              = map.origin_x;
    origin.y              = map.origin_y;
    path.origin           = origin;

    if (path_found) {
      RCLCPP_INFO(rclcpp::get_logger("astar_pather"), "Found path");
      std::vector<igvc::msg::Position> positions;

      int8_t x = goal.x;
      int8_t y = goal.y;

      auto position = igvc::msg::Position();
      position.x    = x;
      position.y    = y;
      positions.push_back(position);

      while (x != start.x || y != start.y) {
        auto [px, py] = came_from[x][y];
        x             = px;
        y             = py;
        auto position = igvc::msg::Position();
        position.x    = x;
        position.y    = y;
        positions.push_back(position);
      }

      std::reverse(positions.begin(), positions.end());
      path.positions = positions;
      return path;

    } else {
      RCLCPP_WARN(rclcpp::get_logger("astar_pather"),
                  "No path found from (%d, %d) to (%d, %d)", start.x, start.y,
                  goal.x, goal.y);
      return path;
    }
  }

private:
  // possible directions of movement
  const std::vector<std::pair<int8_t, int8_t>> KDirections{
      {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

  struct Node final {
    int8_t x, y;

    float g, h;
    inline float f() const { return g + h; }

    Node(int8_t _x, int8_t _y) : x(_x), y(_y), g(0), h(0) {}

    bool operator>(const Node &rhs) { return f() > rhs.f(); }
    bool operator==(const Node &rhs) { return x == rhs.x && y == rhs.y; }
  };

  struct compare final {
    bool operator()(const Node lhs, const Node rhs) {
      return lhs.f() > rhs.f();
    }
  };

  inline float heuristic(Node current, Node goal, float side, float diagonal) {
    float dx = std::abs(current.x - goal.x);
    float dy = std::abs(current.y - goal.y);

    return side * (dx + dy) + (diagonal - 2 * side) * std::min(dx, dy);
  }
};

} // namespace igvc::navigation