#pragma once
#include <string>
#include <vector>

struct Instruction final {
  std::string type; // "MOVE" or "TURN"
  double speed;     // linear for MOVE, angular for TURN
  double duration;  // seconds
};

inline std::vector<Instruction> square_path() {
  return {
      {"MOVE", 0.5, 2.0},  //
      {"TURN", 1.57, 1.0}, //
      {"MOVE", 0.5, 2.0},  //
      {"TURN", 1.57, 1.0}, //
      {"MOVE", 0.5, 2.0},  //
      {"TURN", 1.57, 1.0}, //
      {"MOVE", 0.5, 2.0},  //
      {"TURN", 1.57, 1.0}, //
  };
}