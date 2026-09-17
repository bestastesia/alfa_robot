#pragma once

#include <nlohmann/json.hpp>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace alfa_robot::motion
{
using TransferPath = std::vector<std::array<double, 7>>;

// Only positions are cached. No velocity-continuity or hardware timing claim.
inline TransferPath decodeTransferPath(const nlohmann::json& value)
{
  if (!value.is_array() || value.size() < 2)
    throw std::runtime_error("transfer cache needs at least two states");
  TransferPath path;
  for (const auto& row : value) {
    if (!row.is_array() || row.size() != 7)
      throw std::runtime_error("transfer cache state must have seven joints");
    std::array<double, 7> joints{};
    for (size_t i = 0; i < 7; ++i) {
      if (!row[i].is_number()) throw std::runtime_error("non-numeric cached joint");
      joints[i] = row[i].get<double>();
      if (!std::isfinite(joints[i])) throw std::runtime_error("non-finite cached joint");
    }
    path.push_back(joints);
  }
  return path;
}

inline nlohmann::json readTransferCache(
  const std::string& filename, const nlohmann::json& model, bool allow_missing)
{
  std::ifstream file(filename);
  if (!file) {
    if (allow_missing && !std::filesystem::exists(filename)) return {{"version", 1}, {"model", model},
      {"entries", nlohmann::json::object()}};
    throw std::runtime_error("cannot read transfer cache: " + filename);
  }
  nlohmann::json cache;
  file >> cache;
  if (cache.at("version") != 1 || cache.at("model") != model ||
      !cache.at("entries").is_object())
    throw std::runtime_error("incompatible transfer cache model/schema");
  return cache;
}

inline void writeTransferCache(const std::string& filename, const nlohmann::json& cache)
{
  // One offline writer per file. Readers see either the old or the new complete file.
  const std::string temporary = filename + ".tmp";
  {
    std::ofstream file(temporary);
    file << cache.dump() << '\n';
    file.flush();
    if (!file) throw std::runtime_error("cannot write transfer cache: " + temporary);
  }
  if (std::rename(temporary.c_str(), filename.c_str()) != 0)
    throw std::runtime_error("cannot replace transfer cache: " + filename);
}
}  // namespace alfa_robot::motion
