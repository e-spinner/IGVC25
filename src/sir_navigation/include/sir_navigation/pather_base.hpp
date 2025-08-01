#pragma once

#include "sir/msg/map.hpp"
#include "sir/msg/path.hpp"

#include "sir_navigation/occupancy_map.hpp"

namespace sir::navigation {

class PatherBase {
public:
  PatherBase(std::string name) : m_name(name) {}

  // Required, Pather must give path
  // ------------------------------------------------------------------------
  virtual sir::msg::Path_<std::allocator<void>>
  compute(sir::msg::Position start, sir::msg::Position goal,
          OccupancyMap map) = 0;

  std::string name() { return m_name; }

private:
  std::string m_name;
};

} // namespace sir::navigation
