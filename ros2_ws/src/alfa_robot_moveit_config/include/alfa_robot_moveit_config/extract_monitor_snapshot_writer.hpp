#pragma once

#include <nlohmann/json.hpp>

#include <array>
#include <string>

namespace alfa_robot::motion
{

struct ExtractMonitorFullSelectedSnapshotRequest
{
  double box_front_x = 0.0;
  double scene_y_shift = 0.0;
  double total_elapsed_ms = 0.0;
  std::array<double, 4> stage_elapsed_ms{};
};

struct ExtractMonitorStageSnapshotWriteRequest
{
  const nlohmann::json* snapshot = nullptr;
  std::string context;
  std::string success_message;
};

struct ExtractMonitorStageSnapshotWriteResult
{
  bool success = false;
  std::string message;
  std::string error_log_message;
};

class ExtractMonitorSnapshotWriter
{
public:
  explicit ExtractMonitorSnapshotWriter(std::string path = {});

  const std::string& path() const { return path_; }
  void setPath(std::string path);

  bool write(const nlohmann::json& snapshot, std::string* error = nullptr) const;
  ExtractMonitorStageSnapshotWriteResult writeStageSnapshot(
    const ExtractMonitorStageSnapshotWriteRequest& request) const;
  bool writeFullSelectedSnapshot(
    const ExtractMonitorFullSelectedSnapshotRequest& request,
    std::string* error = nullptr) const;
  std::string writeError(const std::string& error) const;
  std::string writeFailureMessage(const std::string& context) const;
  std::string appendSnapshotPath(const std::string& message) const;
  nlohmann::json readOrEmpty() const;

private:
  std::string path_;
};

}  // namespace alfa_robot::motion
