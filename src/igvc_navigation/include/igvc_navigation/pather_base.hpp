#pragma once

#include "igvc/msg/map.hpp"
#include "igvc/msg/path.hpp"

namespace igvc::navigation {

class PatherBase {
public:
  PatherBase(std::string name) : m_name(name) {}

  // Required, Pather must give path
  // ------------------------------------------------------------------------
  virtual igvc::msg::Path_<std::allocator<void>>
  chart(igvc::cfg::Map &map, igvc::msg::Position &_start,
        igvc::msg::Position &_goal) = 0;

  std::string name() { return m_name; }

private:
  std::string m_name;
};

} // namespace igvc::navigation
