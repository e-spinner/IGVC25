#pragma once

#include "sir_msgs/msg/map.hpp"
#include "sir_msgs/msg/path.hpp"

#include "sir_navigation/occupancy_map.hpp"

namespace sir::navigation {

class PatherBase {
public:
  PatherBase(std::string name) : m_name(name) {}

  // Required, Pather must give path
  // ------------------------------------------------------------------------
  virtual sir_msgs::msg::Path_<std::allocator<void>>
  compute(sir_msgs::msg::Position start, sir_msgs::msg::Position goal,
          OccupancyMap map) = 0;

  std::string name() { return m_name; }

private:
  std::string m_name;
};

} // namespace sir::navigation
