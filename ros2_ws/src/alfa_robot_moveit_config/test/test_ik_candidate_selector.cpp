#include "alfa_robot_moveit_config/optimized_ik_pipeline.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <string>

namespace
{

robot_motion::core::UpdownAwareIkCandidate make_candidate(
  bool legal,
  double score,
  size_t h_index,
  size_t seed_index,
  double h,
  double joint1)
{
  robot_motion::core::UpdownAwareIkCandidate candidate;
  candidate.legal = legal;
  candidate.score = score;
  candidate.h_index = h_index;
  candidate.seed_index = seed_index;
  candidate.h = h;
  candidate.full_joint_names = {
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6",
  };
  candidate.full_joint_values = {
    joint1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };
  return candidate;
}

moveit::core::RobotModelPtr ik_candidate_test_model()
{
  std::string urdf_xml = R"(
<robot name="ik_candidate_robot">
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
      const std::string link = std::string(side) + "_joint" + std::to_string(i);
      urdf_xml +=
        "  <link name=\"" + link + "\"/>\n"
        "  <joint name=\"" + std::string(side) + "_joint" + std::to_string(i) + "\" type=\"revolute\">\n"
        "    <parent link=\"" + parent + "\"/>\n"
        "    <child link=\"" + link + "\"/>\n"
        "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
        "    <axis xyz=\"0 0 1\"/>\n"
        "    <limit lower=\"-3.14159\" upper=\"3.14159\" effort=\"1\" velocity=\"1\"/>\n"
        "  </joint>\n";
      parent = link;
    }
  }
  urdf_xml += "</robot>";

  auto urdf_model = std::make_shared<urdf::Model>();
  const bool urdf_ok = urdf_model->initString(urdf_xml);
  assert(urdf_ok);
  auto srdf_model = std::make_shared<srdf::Model>();
  const bool srdf_ok = srdf_model->initString(*urdf_model, R"(<robot name="ik_candidate_robot"/>)");
  assert(srdf_ok);
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace

int main()
{
  using alfa_robot::motion::IkCandidateSelectionStats;
  using alfa_robot::motion::IkCandidateSelector;
  using alfa_robot::motion::IkCandidateSelectorConfig;
  using alfa_robot::motion::ik_candidate_rejection_counts_json;
  using alfa_robot::motion::joint_limit_margin_cost;
  using alfa_robot::motion::robot_state_from_ik_candidate;

  robot_motion::core::UpdownAwareIkResult result;
  result.candidates.push_back(make_candidate(true, 3.0, 0, 2, 0.3, 0.20));
  result.candidates.push_back(make_candidate(false, 0.1, 0, 0, 0.3, 0.00));
  result.candidates.back().rejection_reason = "tip_error_too_large";
  result.candidates.push_back(make_candidate(true, 1.0, 2, 4, 0.3, 0.00));
  result.candidates.push_back(make_candidate(true, 1.0, 1, 3, 0.3, 0.00));
  result.candidates.push_back(make_candidate(true, 2.0, 0, 5, 0.3, 0.004));
  result.candidates.push_back(make_candidate(false, 4.0, 0, 6, 0.3, 0.30));

  const auto rejection_counts = ik_candidate_rejection_counts_json(result);
  assert(rejection_counts.at("legal") == 4);
  assert(rejection_counts.at("tip_error_too_large") == 1);
  assert(rejection_counts.at("unknown") == 1);

  IkCandidateSelectorConfig config;
  config.dedup_enabled = true;
  config.joint_threshold = 1.0 * M_PI / 180.0;
  config.h_threshold = 0.005;
  config.candidate_limit = 2;

  IkCandidateSelector selector(config);
  IkCandidateSelectionStats stats;
  const auto selected = selector.selectLegalFromResult(result, &stats);

  assert(selected.size() == 2);
  assert(selected[0].score == 1.0);
  assert(selected[0].h_index == 1);
  assert(selected[0].seed_index == 3);
  assert(selected[1].score == 3.0);
  assert(stats.enabled);
  assert(stats.input_count == 4);
  assert(stats.unique_count == 2);
  assert(stats.selected_count == 2);
  assert(stats.removed_count == 2);

  const auto model = ik_candidate_test_model();
  const std::vector<double> limit_weights = {0.5, 3.0, 0.7, 0.5, 1.5, 1.2};
  auto centered = make_candidate(true, 0.0, 0, 0, 0.3, 0.0);
  assert(std::abs(joint_limit_margin_cost(centered, *model, limit_weights, 0.6)) < 1e-12);
  auto joint2_near_limit = centered;
  joint2_near_limit.full_joint_values[1] = 0.9 * M_PI;
  auto joint5_near_limit = centered;
  joint5_near_limit.full_joint_values[4] = 0.9 * M_PI;
  const double joint2_cost = joint_limit_margin_cost(joint2_near_limit, *model, limit_weights, 0.6);
  const double joint5_cost = joint_limit_margin_cost(joint5_near_limit, *model, limit_weights, 0.6);
  assert(joint2_cost > joint5_cost);
  assert(joint5_cost > 0.0);

  moveit::core::RobotState seed(model);
  seed.setToDefaultValues();
  seed.setVariablePosition("updown", 0.2);
  seed.setVariablePosition("left_joint1", 0.1);
  seed.setVariablePosition("right_joint6", -0.1);
  seed.update();

  robot_motion::core::UpdownAwareIkCandidate state_candidate;
  state_candidate.full_joint_names = {
    "updown",
    "left_joint1",
    "not_a_robot_joint",
    "right_joint6",
  };
  state_candidate.full_joint_values = {
    0.7,
    0.4,
    123.0,
    -0.6,
  };
  const auto state = robot_state_from_ik_candidate(
    seed,
    state_candidate,
    nullptr);
  assert(std::abs(state.getVariablePosition("updown") - 0.7) < 1e-9);
  assert(std::abs(state.getVariablePosition("left_joint1") - 0.4) < 1e-9);
  assert(std::abs(state.getVariablePosition("right_joint6") + 0.6) < 1e-9);
  assert(std::abs(state.getVariablePosition("left_joint2")) < 1e-9);

  return 0;
}
