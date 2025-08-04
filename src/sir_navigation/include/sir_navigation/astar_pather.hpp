#pragma once

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "sir_common/types.hpp"

#include "pather_base.hpp"

namespace sir::navigation {

const float SIDE     = sir::cfg::MAP_PRECISION;
const float DIAGONAL = std::sqrt(2 * SIDE);

// https://github.com/JDSherbert/A-Star-Pathfinding/blob/main/Pathfinder.cpp
// https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
class AStarPather final : public PatherBase {
public:
  AStarPather() : PatherBase("astar_pather") {}

  sir::msg::Path_<std::allocator<void>>
  chart(sir::cfg::Map &map, sir::msg::Position &_start,
        sir::msg::Position &_goal) override {

    Node start{_start.x, _start.y};
    Node goal{_goal.x, _goal.y};

    std::priority_queue<Node, std::vector<Node>, compare> open_set;
    // stores g score of each visited, -1 = not visited
    sir::cfg::Map closed_set(-1);
    // stores parent of each Node
    std::array<std::array<std::pair<int8_t, int8_t>, sir::cfg::MAP_SIZE>,
               sir::cfg::MAP_SIZE>
        came_from;

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

            float tentative_g = current.g + distance(dx, dy);

            // check if neighbor has been visited, or if it has, if this is
            // better
            if (closed_set.at(x, y) == -1 ||
                tentative_g < closed_set.at(x, y)) {
              Node neighbor(x, y);
              neighbor.g      = tentative_g;
              neighbor.h      = heuristic(neighbor, goal);
              came_from[x][y] = {current.x, current.y};
              open_set.push(neighbor);
              closed_set.set(x, y, tentative_g);
            }
          }
        }
      }
    }

    // reconstruct path
    auto path             = sir::msg::Path();
    path.time_of_validity = rclcpp::Clock().now();

    if (path_found) {
      RCLCPP_INFO(rclcpp::get_logger("astar_pather"), "Found path");
      std::vector<sir::msg::Position> positions;

      int8_t x = goal.x;
      int8_t y = goal.y;

      auto position = sir::msg::Position();
      position.x    = x;
      position.y    = y;
      positions.push_back(position);

      while (x != start.x || y != start.y) {
        auto [px, py] = came_from[x][y];
        x             = px;
        y             = py;
        auto position = sir::msg::Position();
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

  inline float distance(int8_t dx, int8_t dy) {
    if (dx == 0 || dy == 0)
      return SIDE;
    else
      return DIAGONAL;
  }

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

  inline float heuristic(Node current, Node goal) {
    float dx = std::abs(current.x - goal.x);
    float dy = std::abs(current.y - goal.y);

    return SIDE * (dx + dy) + (DIAGONAL - 2 * SIDE) * std::min(dx, dy);
  }
};

} // namespace sir::navigation