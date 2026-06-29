#include "alfa_robot_moveit_config/extract_monitor_snapshot_writer.hpp"

#include "alfa_robot_moveit_config/extract_monitor_json.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace alfa_robot::motion
{

ExtractMonitorSnapshotWriter::ExtractMonitorSnapshotWriter(std::string path)
: path_(std::move(path))
{}

void ExtractMonitorSnapshotWriter::setPath(std::string path)
{
  path_ = std::move(path);
}

bool ExtractMonitorSnapshotWriter::write(const nlohmann::json& snapshot, std::string* error) const
{
  try {
    const std::filesystem::path path(path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    if (!out) {
      if (error) *error = "failed to open " + path_;
      return false;
    }
    out << snapshot.dump(2) << '\n';
    return true;
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
}

ExtractMonitorStageSnapshotWriteResult ExtractMonitorSnapshotWriter::writeStageSnapshot(
  const ExtractMonitorStageSnapshotWriteRequest& request) const
{
  ExtractMonitorStageSnapshotWriteResult result;
  if (!request.snapshot) {
    result.success = false;
    result.message = writeFailureMessage(request.context);
    result.error_log_message = "extract monitor stage snapshot is null";
    return result;
  }

  std::string error;
  if (write(*request.snapshot, &error)) {
    result.success = true;
    result.message = request.success_message;
    return result;
  }

  result.success = false;
  result.message = writeFailureMessage(request.context);
  result.error_log_message = writeError(error);
  return result;
}

bool ExtractMonitorSnapshotWriter::writeFullSelectedSnapshot(
  const ExtractMonitorFullSelectedSnapshotRequest& request,
  std::string* error) const
{
  const nlohmann::json full_snapshot = extract_monitor_full_selected_snapshot(
    readOrEmpty(),
    request.box_front_x,
    request.scene_y_shift,
    request.total_elapsed_ms,
    request.stage_elapsed_ms);
  return write(full_snapshot, error);
}

std::string ExtractMonitorSnapshotWriter::writeError(const std::string& error) const
{
  return "Failed to write extract monitor snapshot " + path_ + ": " + error;
}

std::string ExtractMonitorSnapshotWriter::writeFailureMessage(const std::string& context) const
{
  return context + ": failed to write snapshot";
}

std::string ExtractMonitorSnapshotWriter::appendSnapshotPath(const std::string& message) const
{
  std::ostringstream out;
  out << message << " snapshot=" << path_;
  return out.str();
}

nlohmann::json ExtractMonitorSnapshotWriter::readOrEmpty() const
{
  try {
    std::ifstream in(path_);
    if (!in) return nlohmann::json::object();
    return nlohmann::json::parse(in);
  } catch (const std::exception&) {
    return nlohmann::json::object();
  }
}

}  // namespace alfa_robot::motion
