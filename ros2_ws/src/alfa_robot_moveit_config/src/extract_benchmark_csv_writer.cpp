#include "alfa_robot_moveit_config/extract_benchmark_csv_writer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace alfa_robot::motion
{

bool ExtractBenchmarkCsvWriter::write(
  const std::string& path,
  const std::vector<ExtractRolloutTiming>& timings,
  const rclcpp::Logger& logger)
{
  if (path.empty()) return true;

  const std::filesystem::path csv_path(path);
  if (!csv_path.parent_path().empty()) {
    std::filesystem::create_directories(csv_path.parent_path());
  }

  std::ofstream out(csv_path, std::ios::out | std::ios::trunc);
  if (!out) {
    RCLCPP_WARN(logger, "Failed to open extract benchmark CSV: %s", path.c_str());
    return false;
  }

  out << "candidate_order,h_index,seed_index,h,ik_score,ik_solve_ms,rollout_ms,interval_ms,success,"
         "loaded_plan_attempted,loaded_plan_success,loaded_plan_ms,loaded_plan_points,"
         "loaded_plan_rank,loaded_pose_distance_sum,loaded_pose_distance_l2,loaded_pose_max_joint_delta,"
         "selected_left_loaded_pose_index,selected_right_loaded_pose_index,"
         "selected_left_loaded_pose_distance,selected_right_loaded_pose_distance,"
         "accepted_steps,failed_steps,final_retreat_x,final_lift_z,final_pitch_deg,"
         "right_final_retreat_x,right_final_lift_z,right_final_pitch_deg,"
         "failure_reason,loaded_plan_failure_reason\n";
  out << std::setprecision(12);

  for (const auto& timing : timings) {
    out << timing.candidate_order << ','
        << timing.h_index << ','
        << timing.seed_index << ','
        << timing.h << ','
        << timing.ik_score << ','
        << timing.ik_solve_ms << ','
        << timing.rollout_ms << ','
        << timing.interval_ms << ','
        << (timing.success ? 1 : 0) << ','
        << (timing.loaded_plan_attempted ? 1 : 0) << ','
        << (timing.loaded_plan_success ? 1 : 0) << ','
        << timing.loaded_plan_ms << ','
        << timing.loaded_plan_points << ','
        << timing.loaded_plan_rank << ','
        << timing.loaded_pose_distance_sum << ','
        << timing.loaded_pose_distance_l2 << ','
        << timing.loaded_pose_max_joint_delta << ','
        << timing.selected_left_loaded_pose_index << ','
        << timing.selected_right_loaded_pose_index << ','
        << timing.selected_left_loaded_pose_distance << ','
        << timing.selected_right_loaded_pose_distance << ','
        << timing.accepted_steps << ','
        << timing.failed_steps << ','
        << timing.final_retreat_x << ','
        << timing.final_lift_z << ','
        << timing.final_pitch_deg << ','
        << timing.right_final_retreat_x << ','
        << timing.right_final_lift_z << ','
        << timing.right_final_pitch_deg << ','
        << '"' << timing.failure_reason << '"' << ','
        << '"' << timing.loaded_plan_failure_reason << '"' << '\n';
  }

  RCLCPP_INFO(logger, "Wrote extract all-legal-IK timing CSV: %s rows=%zu", path.c_str(), timings.size());
  return true;
}

}  // namespace alfa_robot::motion
