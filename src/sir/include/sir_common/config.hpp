#pragma once

#include <cstdlib>
#include <fstream>
#include <iosfwd>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Ported from 7oPR engine
// ------------------------------------------------------------------------

// string to <type> conversion
// ------------------------------------------------------------------------
namespace internal {

template <typename KeyType>
std::optional<KeyType> convert(std::string_view str);

// specializations
template <> inline std::optional<int> convert<int>(std::string_view str) {
  try {
    return std::stoi(std::string(str));
  } catch (...) { return std::nullopt; }
}

template <> inline std::optional<float> convert<float>(std::string_view str) {
  try {
    return std::stof(std::string(str));
  } catch (...) { return std::nullopt; }
}

template <>
inline std::optional<std::string> convert<std::string>(std::string_view str) {
  return std::string(str);
}

template <> inline std::optional<bool> convert<bool>(std::string_view str) {
  std::string lowered;
  for (char c : str) lowered += std::tolower(c);
  return (lowered == "true" || lowered == "1" || lowered == "yes");
}

} // namespace internal

namespace sir::cfg {

/**
 * @brief A simple INI-style configuration loader and accessor.
 *
 * Parses config files with [sections] and key = value pairs,
 * and provides access to config values as string, bool, or int.
 */
class Config {
public:
  Config(const std::string &filepath) {
    file.open(filepath);
    if (!file) throw std::runtime_error("Failed to open config file");

    index_file();
  }

  // Get<type> value from key in section
  // ------------------------------------------------------------------------
  template <typename KeyType>
  const KeyType get(const std::string &section, const std::string &key) const {
    auto str = get_string(section, key);

    // for now, error when no value found, later add fallback
    // if (!str) {
    //   ERROR(false, "Failed to find key: ", B_GRN, key, WHT, " in [", B_BLU,
    //         section, "]");
    //   std::abort();
    // }

    return *internal::convert<KeyType>(*str);
  }

  std::vector<std::string> keys_in_section(const std::string &section) const {
    std::vector<std::string> result;
    auto sec_pos = section_offsets.find(section);
    if (sec_pos == section_offsets.end()) return result;

    file.clear();
    file.seekg(sec_pos->second);

    std::string line;
    while (std::getline(file, line)) {
      line = trim(line);
      if (line.empty() || line[0] == '#') continue;
      if (line.front() == '[' && line.back() == ']') break;

      auto eq_pos = line.find('=');
      if (eq_pos != std::string::npos) {
        result.push_back(trim(line.substr(0, eq_pos)));
      }
    }

    return result;
  }

private:
  mutable std::ifstream file;
  std::map<std::string, std::streampos> section_offsets;

  // Getters
  // ------------------------------------------------------------------------
  std::optional<std::string> get_string(const std::string &section,
                                        const std::string &key) const {
    auto sec_it = section_offsets.find(section);
    if (sec_it == section_offsets.end()) return std::nullopt;

    return search_key_in_section(sec_it->second, section, key);
  }

  // Store line index of each section header
  // ------------------------------------------------------------------------
  void index_file() {
    std::string line;
    std::string current_section;

    while (std::getline(file, line)) {
      line = trim(line);
      if (line.empty() || line[0] == '#') continue;
      if (line.front() == '[' && line.back() == ']') {
        current_section = trim(line.substr(1, line.size() - 2));
        section_offsets[current_section] = file.tellg();
      }
    }

    file.clear();
    file.seekg(0);
  }

  std::optional<std::string>
  search_key_in_section(std::streampos pos, const std::string &section,
                        const std::string &key) const {
    file.clear();
    file.seekg(pos);

    std::string line;
    while (std::getline(file, line)) {
      line = trim(line);
      if (line.empty() || line[0] == '#') continue;
      if (line.front() == '[' && line.back() == ']') break;

      auto eq_pos = line.find('=');
      if (eq_pos == std::string::npos) continue;

      std::string found_key = trim(line.substr(0, eq_pos));
      if (found_key == key) { return trim(line.substr(eq_pos + 1)); }
    }

    // WARN("Failed to find config: ", RED, section, "::", key);
    return std::nullopt;
  }

  static std::string trim(const std::string &s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end   = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
  }
};

} // namespace sir::cfg
