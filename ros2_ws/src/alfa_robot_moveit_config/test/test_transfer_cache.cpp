#include "alfa_robot_moveit_config/transfer_cache.hpp"
#include "alfa_robot_moveit_config/natural_joint_motion.hpp"
#include <cassert>
#include <filesystem>
#include <unistd.h>

int main()
{
  using namespace alfa_robot::motion;
  const std::array<double, 7> positive{3.1, 0, 0, 0, 0, 0, 0};
  const std::array<double, 7> negative{-3.1, 0, 0, 0, 0, 0, 0};
  assert(naturalJointDistanceSquared(positive, negative) < .01);
  assert(naturalJointDistanceSquared(positive, negative, false) > 38.0);
  assert(naturalJointPathLength({positive, negative}, false) > 6.1);
  const auto path = (std::filesystem::temp_directory_path() /
    ("alfa-transfer-" + std::to_string(getpid()) + ".json")).string();
  const nlohmann::json model = {{"urdf", "test model"}, {"srdf", "test semantic"}};
  auto cache = readTransferCache(path, model, true);
  const TransferPath states{{0, 1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6, 7}};
  cache["entries"]["test"] = states;
  writeTransferCache(path, cache);
  auto restored = readTransferCache(path, model, false);
  assert(decodeTransferPath(restored["entries"]["test"]) == states);
  auto must_throw = [](auto action) {
    bool threw = false;
    try { action(); } catch (const std::exception&) { threw = true; }
    assert(threw);
  };
  must_throw([&]() { readTransferCache(path, {{"urdf", "changed"}}, false); });
  must_throw([]() { decodeTransferPath(nlohmann::json::array()); });
  must_throw([]() { decodeTransferPath({{1, 2}, {3, 4}}); });
  must_throw([]() { decodeTransferPath({{nullptr, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}}); });
  must_throw([]() { decodeTransferPath({{"0", 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}}); });
  restored["version"] = 0;
  writeTransferCache(path, restored);
  must_throw([&]() { readTransferCache(path, model, false); });
  std::filesystem::remove(path);
  must_throw([&]() { readTransferCache(path, model, false); });
}
