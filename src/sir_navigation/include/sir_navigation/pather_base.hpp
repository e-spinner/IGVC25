#pragma once

#include "sir/msg/map.hpp"
#include "sir/msg/path.hpp"

namespace sir::navigation {

class PatherBase {
public:
  PatherBase(std::string name) : m_name(name) {}

  // Required, Pather must give path
  // ------------------------------------------------------------------------
  virtual sir::msg::Path_<std::allocator<void>>
  chart(sir::cfg::Map &map, sir::msg::Position &_start,
        sir::msg::Position &_goal) = 0;

  std::string name() { return m_name; }

private:
  std::string m_name;
};

} // namespace sir::navigation
