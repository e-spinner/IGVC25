#pragma once

#include "sir_msgs/msg/grid_update.hpp"
#include "sir_msgs/msg/path.hpp"
#include "sir_msgs/msg/position.hpp"

#include "sir_navigation/obstacle_grid.hpp"

namespace sir::navigation {

class PatherBase {
public:
  PatherBase(std::string name) : m_name(name) {}

  // Required, Pather must give path
  // ------------------------------------------------------------------------
  virtual sir_msgs::msg::Path_<std::allocator<void>> get_path() = 0;

  std::string name() { return m_name; }

  void process_update(const sir_msgs::msg::GridUpdate &update_msg) {
    m_grid.apply_update(update_msg);
  }

  void set_goal(const sir_msgs::msg::Position &goal) { m_goal = goal; }
  void set_pos(const sir_msgs::msg::Position &pos) { m_pos = pos; }

protected:
  ObstacleGrid m_grid;

  sir_msgs::msg::Position m_goal;
  sir_msgs::msg::Position m_pos;

private:
  std::string m_name;
};
} // namespace sir::navigation
