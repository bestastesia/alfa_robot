#include <alfa_robot_moveit_config/wall_sequence.hpp>
#include <alfa_robot_moveit_config/natural_joint_motion.hpp>
#include <cassert>
#include <set>

int main()
{
  using namespace alfa_robot::motion;
  const auto order = wallSequenceOrder();
  assert(order.size() == 25 && order.front() == 20 && order.back() == 4);
  assert(std::set<int>(order.begin(), order.end()).size() == 25);
  for (size_t i = 1; i < order.size(); ++i) {
    assert(order[i] / 5 <= order[i - 1] / 5);
    if (i % 5) assert(order[i] == order[i - 1] + 1);
  }
  const auto rounds = wallTransferRounds();
  assert(rounds.size() == 15);
  std::set<int> round_ids;
  size_t dual_rounds = 0;
  for (const auto& round : rounds) {
    if (round.left_box >= 0) round_ids.insert(round.left_box);
    if (round.right_box >= 0) round_ids.insert(round.right_box);
    dual_rounds += round.dual();
    if (round.dual()) {
      assert(round.left_box / 5 == round.right_box / 5);
      assert(round.left_box % 5 < 2);
      assert(round.right_box % 5 > 2);
    }
  }
  assert(dual_rounds == 10);
  assert(round_ids.size() == 25);
  const std::array<double, 7> zero{};
  auto wrist = zero; wrist[6] = 0.5;
  auto shoulder = zero; shoulder[0] = 0.5;
  assert(naturalJointDistanceSquared(zero, wrist) > naturalJointDistanceSquared(zero, shoulder));
  auto elbow_flip = shoulder; elbow_flip[1] = 0.4;
  auto elbow_flip_to = elbow_flip; elbow_flip_to[1] = -0.4;
  assert(!sameShoulderElbowBranch(elbow_flip, elbow_flip_to));
  assert(naturalJointPath({zero, shoulder}));
  assert(!naturalJointPath({elbow_flip, elbow_flip_to}));
  assert(isBottomBox(0.2, 0.4, 0));
  assert(isBottomBox(1.2, 0.4, 1));
  assert(!isBottomBox(0.61, 0.4, 0));
  const auto tries = wallGraspAttempts(false, "auto");
  assert(tries.size() == 4);
  assert(!tries[0].first && tries[0].second == "left");
  assert(!tries[1].first && tries[1].second == "right");
  assert(tries[2].first && tries[2].second == "left");
  assert(tries[3].first && tries[3].second == "right");
  assert(wallGraspAttempts(true, "auto").size() == 2);
  assert(wallGraspAttempts(true, "left").front().first);
  assert(wallGraspAttempts(false, "right").size() == 2);
  assert(wallGraspAttempts(false, "auto", true) == wallGraspAttempts(true, "auto"));
  assert(wallGraspAttempts(false, "left", true) == wallGraspAttempts(true, "left"));
  assert(wallGraspAttempts(true, "right", true) == wallGraspAttempts(true, "right"));
  const Eigen::Vector3d size(0.3, 0.4, 0.5), center(1, 2, 3);
  for (bool top : {false, true}) {
    const auto contact = wallContactPose(center, size, 1e-6, top);
    const auto offset = wallContactPose(Eigen::Vector3d::Zero(), size, 1e-6, top).inverse();
    const Eigen::Isometry3d box = contact * offset;
    assert(box.translation().isApprox(center));
    assert(box.linear().isApprox(Eigen::Matrix3d::Identity()));
    const Eigen::Vector3d axis = top ? Eigen::Vector3d(0, 0, -1) : Eigen::Vector3d(1, 0, 0);
    assert(contact.linear().col(2).isApprox(axis));
    // Rear placement must account for the box offset/extent under either suction mode,
    // including folded transport orientations; TCP-behind alone is insufficient.
    for (double angle : {0.0, 0.7, -1.2}) {
      Eigen::Isometry3d transport = contact;
      transport.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix() * contact.linear();
      const auto rear = wallRearPlacementPose(transport, offset, size, -0.5, 0.02);
      assert(boxBehindChassis(rear * offset, size, -0.5));
      double max_x = -1e9;
      for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3d corner((i & 1) ? size.x()/2 : -size.x()/2,
          (i & 2) ? size.y()/2 : -size.y()/2, (i & 4) ? size.z()/2 : -size.z()/2);
        max_x = std::max(max_x, (rear * offset * corner).x());
      }
      assert(std::abs(max_x - (-0.52)) < 1e-9);
    }
    // Eight non-cubic corners remain unchanged at attachment.
    for (int i = 0; i < 8; ++i) {
      Eigen::Vector3d corner((i & 1) ? size.x()/2 : -size.x()/2,
        (i & 2) ? size.y()/2 : -size.y()/2, (i & 4) ? size.z()/2 : -size.z()/2);
      assert((box * corner).isApprox(center + corner));
    }
  }
  auto box = Eigen::Isometry3d::Identity();
  box.translation().x() = -0.7;
  assert(boxBehindChassis(box, size, -0.5));
  box.translation().x() = -0.65;
  assert(!boxBehindChassis(box, size, -0.5));  // center behind is insufficient
  box.translation().x() = -0.7;
  box.linear() = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitY()).toRotationMatrix();
  assert(!boxBehindChassis(box, size, -0.5));  // rotated extent matters
}
