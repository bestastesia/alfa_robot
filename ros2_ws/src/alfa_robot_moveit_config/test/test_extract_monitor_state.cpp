#include "alfa_robot_moveit_config/extract_monitor_state.hpp"

#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <memory>
#include <string>
#include <vector>

namespace
{

moveit::core::RobotModelPtr monitor_seed_test_model()
{
  std::string urdf_xml = R"(
<robot name="monitor_seed_robot">
  <link name="base_link"/>
  <link name="updown_link"/>
  <joint name="updown" type="prismatic">
    <parent link="base_link"/>
    <child link="updown_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
)";
  for (const auto side : {"left", "right"}) {
    std::string parent = "updown_link";
    for (int i = 1; i <= 6; ++i) {
      const std::string link = std::string(side) + "_v5_link" + std::to_string(i);
      urdf_xml +=
        "  <link name=\"" + link + "\"/>\n"
        "  <joint name=\"" + std::string(side) + "_v5_joint" + std::to_string(i) + "\" type=\"revolute\">\n"
        "    <parent link=\"" + parent + "\"/>\n"
        "    <child link=\"" + link + "\"/>\n"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
        "    <axis xyz=\"0 0 1\"/>\n"
        "    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>\n"
        "  </joint>\n";
      parent = link;
    }
  }
  urdf_xml += "</robot>";

  auto urdf_model = std::make_shared<urdf::Model>();
  assert(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  assert(srdf_model->initString(*urdf_model, R"(<robot name="monitor_seed_robot"/>)"));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::ExtractMonitorArmSeed;
  using alfa_robot::motion::ExtractMonitorExtractStageMessageRequest;
  using alfa_robot::motion::ExtractMonitorFinalStageMessageRequest;
  using alfa_robot::motion::ExtractMonitorFullRunResult;
  using alfa_robot::motion::ExtractMonitorController;
  using alfa_robot::motion::ExtractMonitorIkStageMessageRequest;
  using alfa_robot::motion::ExtractMonitorInitialStateRequest;
  using alfa_robot::motion::ExtractMonitorLoadedStageMessageRequest;
  using alfa_robot::motion::ExtractMonitorPhase;
  using alfa_robot::motion::ExtractMonitorStage;
  using alfa_robot::motion::ExtractMonitorStageCallbacks;
  using alfa_robot::motion::AttachedBoxSpec;
  using alfa_robot::motion::extract_monitor_candidate_for_timing;
  using alfa_robot::motion::extract_monitor_candidate_state_for_timing;
  using alfa_robot::motion::extract_monitor_extract_stage_message;
  using alfa_robot::motion::extract_monitor_final_stage_message;
  using alfa_robot::motion::extract_monitor_ik_stage_message;
  using alfa_robot::motion::extract_monitor_loaded_stage_message;
  using alfa_robot::motion::extract_monitor_prefix;
  using alfa_robot::motion::extract_monitor_worker_count;
  using alfa_robot::motion::make_extract_monitor_initial_state;
  using alfa_robot::motion::populate_extract_monitor_candidate_states;
  using alfa_robot::motion::run_extract_monitor_candidate_tasks;
  using alfa_robot::motion::select_extract_monitor_final_timing;
  using alfa_robot::motion::summarize_extract_monitor_timings;
  using alfa_robot::motion::summarize_loaded_plan_timings;
  using alfa_robot::motion::extract_monitor_next_phase_after;
  using alfa_robot::motion::extract_monitor_phase_before_running;
  using alfa_robot::motion::extract_monitor_stage_for_phase;
  using alfa_robot::motion::make_extract_monitor_joint_state;
  using alfa_robot::motion::run_extract_monitor_full_sequence;
  using alfa_robot::motion::run_extract_monitor_stage;

  assert(extract_monitor_stage_for_phase(ExtractMonitorPhase::ReadyForIk) == ExtractMonitorStage::Ik);
  assert(extract_monitor_stage_for_phase(ExtractMonitorPhase::ReadyForExtract) == ExtractMonitorStage::Extract);
  assert(extract_monitor_stage_for_phase(ExtractMonitorPhase::ReadyForLoaded) == ExtractMonitorStage::Loaded);
  assert(extract_monitor_stage_for_phase(ExtractMonitorPhase::ReadyForFinal) == ExtractMonitorStage::Final);
  assert(extract_monitor_stage_for_phase(ExtractMonitorPhase::Done) == ExtractMonitorStage::Ik);
  assert(extract_monitor_phase_before_running(ExtractMonitorPhase::Done) == ExtractMonitorPhase::ReadyForIk);
  assert(extract_monitor_phase_before_running(ExtractMonitorPhase::ReadyForLoaded) == ExtractMonitorPhase::ReadyForLoaded);

  assert(extract_monitor_next_phase_after(ExtractMonitorStage::Ik) == ExtractMonitorPhase::ReadyForExtract);
  assert(extract_monitor_next_phase_after(ExtractMonitorStage::Extract) == ExtractMonitorPhase::ReadyForLoaded);
  assert(extract_monitor_next_phase_after(ExtractMonitorStage::Loaded) == ExtractMonitorPhase::ReadyForFinal);
  assert(extract_monitor_next_phase_after(ExtractMonitorStage::Final) == ExtractMonitorPhase::Done);

  std::vector<std::string> calls;
  double last_elapsed = 0.0;
  ExtractMonitorStageCallbacks callbacks;
  callbacks.ik = [&](std::string* message) {
    calls.push_back("ik");
    last_elapsed = 1.0;
    if (message) *message = "ik ok";
    return true;
  };
  callbacks.extract = [&](std::string* message) {
    calls.push_back("extract");
    last_elapsed = 2.0;
    if (message) *message = "extract ok";
    return true;
  };
  callbacks.loaded = [&](std::string* message) {
    calls.push_back("loaded");
    last_elapsed = 3.0;
    if (message) *message = "loaded ok";
    return true;
  };
  callbacks.final = [&](std::string* message) {
    calls.push_back("final");
    last_elapsed = 4.0;
    if (message) *message = "final ok";
    return true;
  };

  std::string message;
  double elapsed = 0.0;
  assert(run_extract_monitor_stage(
    ExtractMonitorStage::Loaded, callbacks, [&] { return last_elapsed; }, &elapsed, &message));
  assert(calls.size() == 1 && calls.back() == "loaded");
  assert(elapsed == 3.0);
  assert(message == "loaded ok");

  calls.clear();
  last_elapsed = 0.0;
  const ExtractMonitorFullRunResult result = run_extract_monitor_full_sequence(callbacks, [&] { return last_elapsed; });
  assert(result.success);
  assert(calls.size() == 4);
  assert(calls[0] == "ik");
  assert(calls[1] == "extract");
  assert(calls[2] == "loaded");
  assert(calls[3] == "final");
  assert(result.stage_elapsed_ms[0] == 1.0);
  assert(result.stage_elapsed_ms[1] == 2.0);
  assert(result.stage_elapsed_ms[2] == 3.0);
  assert(result.stage_elapsed_ms[3] == 4.0);
  assert(result.message.find("完整流程完成") != std::string::npos);

  callbacks.extract = [&](std::string* fail_message) {
    calls.push_back("extract_fail");
    last_elapsed = 20.0;
    if (fail_message) *fail_message = "bad extract";
    return false;
  };
  calls.clear();
  const auto failed = run_extract_monitor_full_sequence(callbacks, [&] { return last_elapsed; });
  assert(!failed.success);
  assert(failed.message.find("完整流程失败在抽离阶段") != std::string::npos);

  ExtractMonitorController controller;
  calls.clear();
  callbacks.extract = [&](std::string* message) {
    calls.push_back("extract");
    last_elapsed = 2.0;
    if (message) *message = "extract ok";
    return true;
  };
  assert(controller.phase() == ExtractMonitorPhase::ReadyForIk);
  assert(controller.runNext(callbacks, [&] { return last_elapsed; }, &message));
  assert(controller.phase() == ExtractMonitorPhase::ReadyForExtract);
  assert(controller.runNext(callbacks, [&] { return last_elapsed; }, &message));
  assert(controller.phase() == ExtractMonitorPhase::ReadyForLoaded);
  assert(controller.runNext(callbacks, [&] { return last_elapsed; }, &message));
  assert(controller.phase() == ExtractMonitorPhase::ReadyForFinal);
  assert(controller.runNext(callbacks, [&] { return last_elapsed; }, &message));
  assert(controller.phase() == ExtractMonitorPhase::Done);
  assert(controller.runNext(callbacks, [&] { return last_elapsed; }, &message));
  assert(controller.phase() == ExtractMonitorPhase::ReadyForExtract);

  controller.reset();
  const auto full_from_controller = controller.runFull(callbacks, [&] { return last_elapsed; });
  assert(full_from_controller.success);
  assert(controller.phase() == ExtractMonitorPhase::ReadyForIk);

  AttachedBoxSpec left_box;
  left_box.id = "left_box";
  AttachedBoxSpec right_box;
  right_box.id = "right_box";
  auto state = make_extract_monitor_initial_state(
    6,
    8,
    left_box,
    right_box,
    {},
    {});
  assert(extract_monitor_prefix(6, 8) == "extract_monitor_L6_R8");
  assert(state.left_box_id == 6);
  assert(state.right_box_id == 8);
  assert(state.prefix == "extract_monitor_L6_R8");
  assert(state.left_box.id == "left_box");
  assert(state.right_box.id == "right_box");
  assert(!state.seed_state);
  assert(!state.loaded_start_state);

  const auto seed_model = monitor_seed_test_model();
  const ExtractMonitorArmSeed arm_seed{
    {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
    {-0.1, -0.2, -0.3, -0.4, -0.5, -0.6},
    0.35};
  const auto seed_state = make_extract_monitor_joint_state(seed_model, nullptr, arm_seed);
  assert(seed_state.getVariablePosition("updown") == 0.35);
  assert(seed_state.getVariablePosition("left_v5_joint1") == 0.1);
  assert(seed_state.getVariablePosition("left_v5_joint6") == 0.6);
  assert(seed_state.getVariablePosition("right_v5_joint1") == -0.1);
  assert(seed_state.getVariablePosition("right_v5_joint6") == -0.6);

  const auto initialized_state = make_extract_monitor_initial_state(
    ExtractMonitorInitialStateRequest{
      7,
      9,
      left_box,
      right_box,
      seed_model,
      nullptr,
      arm_seed,
      ExtractMonitorArmSeed{
        {1.1, 1.2, 1.3, 1.4, 1.5, 1.6},
        {-1.1, -1.2, -1.3, -1.4, -1.5, -1.6},
        0.45}});
  assert(initialized_state.left_box_id == 7);
  assert(initialized_state.right_box_id == 9);
  assert(initialized_state.prefix == "extract_monitor_L7_R9");
  assert(initialized_state.seed_state);
  assert(initialized_state.loaded_start_state);
  assert(initialized_state.seed_state->getVariablePosition("updown") == 0.35);
  assert(initialized_state.loaded_start_state->getVariablePosition("updown") == 0.45);
  assert(initialized_state.loaded_start_state->getVariablePosition("left_v5_joint1") == 1.1);
  assert(initialized_state.loaded_start_state->getVariablePosition("right_v5_joint6") == -1.6);

  state.legal_candidates.resize(3);
  size_t built_count = 0;
  populate_extract_monitor_candidate_states(
    state,
    [&](const ik_benchmark::UpdownAwareIkCandidate&) {
      ++built_count;
      return moveit::core::RobotStatePtr{};
    });
  assert(built_count == 3);
  assert(state.candidate_states.size() == 3);
  populate_extract_monitor_candidate_states(state, {});
  assert(state.candidate_states.empty());

  assert(extract_monitor_worker_count(0, 8) == 0);
  assert(extract_monitor_worker_count(3, 8) == 3);
  assert(extract_monitor_worker_count(3, 0) == 1);
  assert(extract_monitor_worker_count(10, 4) == 4);

  const size_t used_workers = run_extract_monitor_candidate_tasks(
    state,
    2,
    [](size_t index, const ik_benchmark::UpdownAwareIkCandidate&) {
      alfa_robot::motion::ExtractRolloutTiming timing;
      timing.candidate_order = index;
      timing.success = index != 1;
      timing.failure_reason = timing.success ? "" : "synthetic_failure";
      return timing;
    });
  assert(used_workers == 2);
  assert(state.timings.size() == 3);
  assert(state.timings[0].candidate_order == 0);
  assert(state.timings[1].candidate_order == 1);
  assert(state.timings[1].failure_reason == "synthetic_failure");

  alfa_robot::motion::ExtractRolloutTiming candidate_timing;
  candidate_timing.candidate_order = 1;
  assert(extract_monitor_candidate_for_timing(state, candidate_timing) == &state.legal_candidates[1]);
  assert(extract_monitor_candidate_state_for_timing(state, candidate_timing) == nullptr);
  state.candidate_states.resize(3);
  state.candidate_states[1] = std::make_shared<moveit::core::RobotState>(seed_state);
  assert(extract_monitor_candidate_state_for_timing(state, candidate_timing) == state.candidate_states[1]);
  candidate_timing.candidate_order = 99;
  assert(extract_monitor_candidate_for_timing(state, candidate_timing) == nullptr);
  assert(extract_monitor_candidate_state_for_timing(state, candidate_timing) == nullptr);

  std::vector<alfa_robot::motion::ExtractRolloutTiming> timings(4);
  moveit::core::RobotStatePtr fake_final_state(
    reinterpret_cast<moveit::core::RobotState*>(0x1),
    [](moveit::core::RobotState*) {});
  timings[0].success = true;
  timings[0].final_state = fake_final_state;
  timings[1].success = true;
  timings[1].failure_reason = "missing_final_state";
  timings[2].failure_reason = "collision";
  const auto timing_summary = summarize_extract_monitor_timings(timings);
  assert(timing_summary.success_count == 1);
  assert(timing_summary.success_indices.size() == 1);
  assert(timing_summary.success_indices[0] == 0);
  assert(timing_summary.failure_counts.at("missing_final_state") == 1);
  assert(timing_summary.failure_counts.at("collision") == 1);
  assert(timing_summary.failure_counts.at("unknown") == 1);

  timings[0].loaded_plan_attempted = true;
  timings[0].loaded_plan_success = true;
  timings[1].loaded_plan_attempted = true;
  timings[1].loaded_plan_failure_reason = "rrt_failed";
  timings[2].loaded_plan_attempted = true;
  const auto loaded_summary = summarize_loaded_plan_timings(timings, {0, 1, 2, 99});
  assert(loaded_summary.attempted_count == 3);
  assert(loaded_summary.success_count == 1);
  assert(loaded_summary.attempted_indices.size() == 3);
  assert(loaded_summary.success_indices.size() == 1);
  assert(loaded_summary.success_indices[0] == 0);
  assert(loaded_summary.failure_counts.at("rrt_failed") == 1);
  assert(loaded_summary.failure_counts.at("unknown") == 1);

  assert(extract_monitor_ik_stage_message(64, 301, 512, 12.5, "/tmp/snapshot.json") ==
         "IK阶段完成: unique=64 legal=301 trials=512 elapsed=12.5ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_ik_stage_message(
    ExtractMonitorIkStageMessageRequest{64, 301, 512, 12.5, "/tmp/snapshot.json"}) ==
         "IK阶段完成: unique=64 legal=301 trials=512 elapsed=12.5ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_extract_stage_message(8, 64, 16, 22.0, "/tmp/snapshot.json") ==
         "抽离阶段完成: success=8/64 workers=16 elapsed=22ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_extract_stage_message(
    ExtractMonitorExtractStageMessageRequest{8, 64, 16, 22.0, "/tmp/snapshot.json"}) ==
         "抽离阶段完成: success=8/64 workers=16 elapsed=22ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_loaded_stage_message(1, 8, 8, 33.0, "/tmp/snapshot.json") ==
         "负重规划阶段完成: success=1 attempted=8 candidates=8 elapsed=33ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_loaded_stage_message(
    ExtractMonitorLoadedStageMessageRequest{1, 8, 8, 33.0, "/tmp/snapshot.json"}) ==
         "负重规划阶段完成: success=1 attempted=8 candidates=8 elapsed=33ms snapshot=/tmp/snapshot.json");
  timings[0].candidate_order = 7;
  timings[0].loaded_plan_rank = 2;
  timings[0].loaded_plan_trajectory_distance = 1.25;
  assert(extract_monitor_final_stage_message(timings[0], 44.0, "/tmp/snapshot.json") ==
         "最终方案已选择: candidate_order=7 loaded_rank=2 trajectory_distance=1.25 elapsed=44ms snapshot=/tmp/snapshot.json");
  assert(extract_monitor_final_stage_message(
    ExtractMonitorFinalStageMessageRequest{&timings[0], 44.0, "/tmp/snapshot.json"}) ==
         "最终方案已选择: candidate_order=7 loaded_rank=2 trajectory_distance=1.25 elapsed=44ms snapshot=/tmp/snapshot.json");

  std::vector<alfa_robot::motion::ExtractRolloutTiming> final_timings(4);
  final_timings[0].loaded_plan_success = true;
  final_timings[0].final_state = fake_final_state;
  final_timings[0].candidate_order = 0;
  final_timings[1].loaded_plan_success = true;
  final_timings[1].final_state = fake_final_state;
  final_timings[1].candidate_order = 1;
  final_timings[2].loaded_plan_success = false;
  final_timings[2].candidate_order = 2;

  auto* preferred = select_extract_monitor_final_timing(
    final_timings,
    [](const alfa_robot::motion::ExtractRolloutTiming& timing) {
      return timing.candidate_order == 1;
    });
  assert(preferred == &final_timings[1]);

  auto* fallback = select_extract_monitor_final_timing(
    final_timings,
    [](const alfa_robot::motion::ExtractRolloutTiming&) {
      return false;
    });
  assert(fallback == &final_timings[0]);

  final_timings[0].final_state.reset();
  final_timings[1].final_state.reset();
  auto* missing_ready_state = select_extract_monitor_final_timing(final_timings, {});
  assert(missing_ready_state == nullptr);

  return 0;
}
