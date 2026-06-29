#include "alfa_robot_moveit_config/extract_monitor_snapshot_writer.hpp"

#include <array>
#include <cassert>
#include <filesystem>
#include <string>

int main()
{
  const auto path = std::filesystem::temp_directory_path() /
    "alfa_extract_monitor_snapshot_writer_test" / "nested" / "snapshot.json";
  std::filesystem::remove_all(path.parent_path().parent_path());

  alfa_robot::motion::ExtractMonitorSnapshotWriter writer(path.string());
  const std::string formatted_error = writer.writeError("disk full");
  assert(formatted_error.find(path.string()) != std::string::npos);
  assert(formatted_error.find("disk full") != std::string::npos);
  assert(writer.writeFailureMessage("extract monitor IK") ==
    "extract monitor IK: failed to write snapshot");

  std::string error;
  const bool ok = writer.write({{"phase", "ik_candidates"}, {"count", 3}}, &error);
  assert(ok);
  assert(error.empty());

  const auto loaded = writer.readOrEmpty();
  assert(loaded.is_object());
  assert(loaded.at("phase") == "ik_candidates");
  assert(loaded.at("count") == 3);

  const nlohmann::json stage_snapshot{{"phase", "extract_success"}, {"count", 8}};
  const auto stage_result = writer.writeStageSnapshot(
    alfa_robot::motion::ExtractMonitorStageSnapshotWriteRequest{
      &stage_snapshot,
      "extract monitor extract",
      "抽离阶段完成"});
  assert(stage_result.success);
  assert(stage_result.message == "抽离阶段完成");
  assert(stage_result.error_log_message.empty());
  const auto stage_loaded = writer.readOrEmpty();
  assert(stage_loaded.at("phase") == "extract_success");
  assert(stage_loaded.at("count") == 8);

  const auto null_stage_result = writer.writeStageSnapshot(
    alfa_robot::motion::ExtractMonitorStageSnapshotWriteRequest{
      nullptr,
      "extract monitor final",
      "最终方案完成"});
  assert(!null_stage_result.success);
  assert(null_stage_result.message == "extract monitor final: failed to write snapshot");
  assert(null_stage_result.error_log_message == "extract monitor stage snapshot is null");

  const bool full_ok = writer.writeFullSelectedSnapshot(
    alfa_robot::motion::ExtractMonitorFullSelectedSnapshotRequest{
      0.925,
      -0.4,
      123.0,
      std::array<double, 4>{1.0, 2.0, 3.0, 4.0}},
    &error);
  assert(full_ok);
  const auto full_loaded = writer.readOrEmpty();
  assert(full_loaded.at("phase") == "full_selected");
  assert(full_loaded.at("phase_label") == "完整流程最终采用方案");
  assert(full_loaded.at("box_front_x") == 0.925);
  assert(full_loaded.at("scene_y_shift") == -0.4);
  assert(full_loaded.at("elapsed_ms") == 123.0);
  assert(full_loaded.at("ik_elapsed_ms") == 1.0);
  assert(full_loaded.at("extract_elapsed_ms") == 2.0);
  assert(full_loaded.at("loaded_elapsed_ms") == 3.0);
  assert(full_loaded.at("final_elapsed_ms") == 4.0);

  const auto message = writer.appendSnapshotPath("done");
  assert(message.find("done snapshot=") == 0);
  assert(message.find(path.string()) != std::string::npos);

  std::filesystem::remove_all(path.parent_path().parent_path());
  return 0;
}
