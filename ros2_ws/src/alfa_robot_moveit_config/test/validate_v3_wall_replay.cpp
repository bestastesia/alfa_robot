// Independent acceptance checker: reconstruct the published scene and interpolate
// the published joints, without calling the demo's planner or collision helpers.
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <set>
#include <fstream>
#include <iostream>
#include <stdexcept>

using Json = nlohmann::json;
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
std::string read(const char* path) { std::ifstream in(path); require(bool(in), path); return {std::istreambuf_iterator<char>(in), {}}; }
Eigen::Vector3d vec(const Json& j) { return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()}; }

void validateCycle(const Json& task, const moveit::core::RobotModelPtr& model)
{
    require(task.at("success"), "requires a successful cycle");
    const bool release = task.at("release_after_transfer");
    planning_scene::PlanningScene scene(model);
    require(task.at("collision_inset").get<double>() == 0, "box geometry must not be shrunk");
    const auto size = vec(task.at("box_size"));
    const auto center = vec(task.at("box_center"));
    auto add = [&](const std::string& id, const Eigen::Vector3d& p, const Eigen::Vector3d& dimensions) {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = model->getModelFrame(); obj.id = id; obj.operation = obj.ADD;
      shape_msgs::msg::SolidPrimitive box; box.type = box.BOX;
      box.dimensions = {dimensions.x(), dimensions.y(), dimensions.z()};
      geometry_msgs::msg::Pose pose; pose.orientation.w = 1;
      pose.position.x = p.x(); pose.position.y = p.y(); pose.position.z = p.z();
      obj.primitives.push_back(box); obj.primitive_poses.push_back(pose);
      require(scene.processCollisionObjectMsg(obj), "cannot add " + id);
    };
    require(task.at("environment").at("enabled"), "environment must remain enabled");
    for (const auto& b : task.at("environment").at("boxes")) add(b.at("id"), vec(b.at("center")), vec(b.at("size")));
    size_t neighbor = 0;
    for (const auto& c : task.at("neighbor_centers")) add("neighbor_" + std::to_string(neighbor++), vec(c), size);
    add("target", center, size);
    const auto names = task.at("joint_names").get<std::vector<std::string>>();
    const auto frames = task.at("frames");
    require(frames.size() > 2 && !frames.front().at("box_attached").get<bool>(), "missing empty-arm start");
    require(frames.front().at("joints") == task.at("initial_joints"), "start teleported from home");
    const std::string tool = task.at("tool_link"), side = task.at("side");
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = vec(task.at("tool_to_box_center"));
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) offset.linear()(i,j) = task.at("tool_to_box_rotation")[i][j];
    moveit::core::RobotState state(model); state.setToDefaultValues();
    bool attached = false;
    size_t checks = 0, attachments = 0, releases = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
      const auto q = frames[f].at("joints").get<std::vector<double>>();
      require(q.size() == names.size(), "invalid frame joint count");
      const auto prev = f ? frames[f-1].at("joints").get<std::vector<double>>() : q;
      const bool now_attached = frames[f].at("box_attached");
      const std::string stage = frames[f].at("stage");
      require(stage.find("SNAPSHOT") == std::string::npos && stage.find("UNCONNECTED") == std::string::npos, "diagnostics in executable frames");
      if (now_attached != attached) {
        require(q == prev, "attachment/release teleported");
        for (size_t j = 0; j < names.size(); ++j) state.setVariablePosition(names[j], q[j]);
        state.update(true);
        const Eigen::Isometry3d box = state.getGlobalLinkTransform(tool) * offset;
        if (now_attached) {
          require(stage == "attach_box" && releases == 0, "unexpected attachment");
          require((box.translation() - center).norm() < 1e-6 &&
            (box.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6, "box jumped/rotated at attachment");
          scene.getWorldNonConst()->removeObject("target");
          EigenSTL::vector_Isometry3d poses{offset};
          state.attachBody("target", Eigen::Isometry3d::Identity(), {shapes::ShapeConstPtr(new shapes::Box(size.x(),size.y(),size.z()))},
            poses, std::set<std::string>{tool, side + "_joint7"}, tool);
          ++attachments;
        } else {
          require(release && stage == "release_box" && f && frames[f-1].at("stage") == "rear_placement", "unexpected release");
          // Verify all eight corners independently of the planner's extent helper.
          for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3d corner((i & 1) ? size.x()/2 : -size.x()/2,
              (i & 2) ? size.y()/2 : -size.y()/2, (i & 4) ? size.z()/2 : -size.z()/2);
            require((box * corner).x() <= task.at("chassis_rear_x").get<double>() - .01 + 1e-8, "box not fully behind chassis at release");
          }
          state.clearAttachedBody("target"); ++releases;
        }
        attached = now_attached;
      }
      require(frames[f].at("box_visible").get<bool>() == (releases == 0), "box disappeared before release or reappeared");
      size_t steps = 1;
      for (size_t j = 0; j < names.size(); ++j)
        steps = std::max(steps, static_cast<size_t>(std::ceil(std::abs(q[j]-prev[j]) /
          (names[j] == "updown" ? .0025 : .5 * std::acos(-1.) / 180.))));
      for (size_t step = 1; step <= steps; ++step) {
        for (size_t j = 0; j < names.size(); ++j)
          state.setVariablePosition(names[j], prev[j] + (q[j]-prev[j])*step/steps);
        state.update(true);
        require(state.satisfiesBounds(), "joint limit at frame " + std::to_string(f));
        collision_detection::CollisionRequest request; request.contacts = true; request.max_contacts = 10;
        collision_detection::CollisionResult result;
        auto acm = scene.getAllowedCollisionMatrix();
        // Only the true suction contact may touch the target; never exempt neighbors/environment.
        if (!attached && (state.getGlobalLinkTransform(tool) * offset).translation().isApprox(center, 1e-6)) {
          acm.setEntry("target", tool, true); acm.setEntry("target", side + "_joint7", true);
        }
        scene.checkCollision(request, result, state, acm); ++checks;
        if (result.collision) {
          std::string pairs;
          for (const auto& contact : result.contacts) pairs += contact.first.first + "<->" + contact.first.second + " ";
          throw std::runtime_error("collision frame=" + std::to_string(f) + " stage=" + stage + " " + pairs);
        }
      }
    }
    require(attachments == 1 && releases == (release ? 1u : 0u) && attached == !release, "wrong payload lifecycle");
    if (release) require(frames.back().at("stage") == "release_box", "cycle must end at rear release, not reset home");
    std::cout << "PASS independent full-scene replay: " << frames.size() << " frames, " << checks
              << " collision/bounds probes (0.5deg / 2.5mm), attachment continuous, payload "
              << (release ? "released behind chassis, final posture retained" : "retained") << "\n";
}


void validateDualCycle(const Json& task, const moveit::core::RobotModelPtr& model)
{
    require(task.at("success") && task.at("dual"), "requires a successful dual cycle");
    planning_scene::PlanningScene scene(model);
    const auto size = vec(task.at("box_size"));
    auto add = [&](const std::string& id, const Eigen::Vector3d& p) {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = model->getModelFrame(); obj.id = id; obj.operation = obj.ADD;
      shape_msgs::msg::SolidPrimitive box; box.type = box.BOX;
      box.dimensions = {size.x(), size.y(), size.z()};
      geometry_msgs::msg::Pose pose; pose.orientation.w = 1;
      pose.position.x = p.x(); pose.position.y = p.y(); pose.position.z = p.z();
      obj.primitives.push_back(box); obj.primitive_poses.push_back(pose);
      require(scene.processCollisionObjectMsg(obj), "cannot add " + id);
    };
    for (const auto& b : task.at("environment").at("boxes")) {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = model->getModelFrame(); obj.id = b.at("id"); obj.operation = obj.ADD;
      shape_msgs::msg::SolidPrimitive box; box.type = box.BOX;
      const auto dimensions = vec(b.at("size")); box.dimensions = {dimensions.x(), dimensions.y(), dimensions.z()};
      const auto center = vec(b.at("center")); geometry_msgs::msg::Pose pose; pose.orientation.w = 1;
      pose.position.x = center.x(); pose.position.y = center.y(); pose.position.z = center.z();
      obj.primitives.push_back(box); obj.primitive_poses.push_back(pose);
      require(scene.processCollisionObjectMsg(obj), "cannot add environment object");
    }
    const auto frames = task.at("frames");
    require(frames.size() > 2 && frames.front().at("carried_boxes").size() == 2, "missing dual payload metadata");
    struct Payload { int id; std::string object; std::string side; std::string tool; Eigen::Vector3d center; Eigen::Isometry3d offset; };
    std::vector<Payload> payloads;
    for (const auto& carried : frames.front().at("carried_boxes")) {
      Payload payload;
      payload.id = carried.at("box_id"); payload.object = "dual_target_" + std::to_string(payload.id);
      payload.side = carried.at("side"); payload.tool = carried.at("tool_link");
      payload.center = vec(carried.at("box_center")); payload.offset = Eigen::Isometry3d::Identity();
      payload.offset.translation() = vec(carried.at("tool_to_box_center"));
      for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        payload.offset.linear()(i,j) = carried.at("tool_to_box_rotation")[i][j];
      payloads.push_back(payload);
    }
    size_t neighbor = 0;
    for (const auto& center_json : task.at("neighbor_centers")) {
      const auto center = vec(center_json); bool target = false;
      for (const auto& payload : payloads) target = target || center.isApprox(payload.center, 1e-6);
      if (!target) add("neighbor_" + std::to_string(neighbor++), center);
    }
    for (const auto& payload : payloads) add(payload.object, payload.center);

    const auto names = task.at("joint_names").get<std::vector<std::string>>();
    moveit::core::RobotState state(model); state.setToDefaultValues();
    bool attached = false; size_t attachments = 0, releases = 0, checks = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
      const auto q = frames[f].at("joints").get<std::vector<double>>();
      const auto prev = f ? frames[f-1].at("joints").get<std::vector<double>>() : q;
      require(q.size() == names.size(), "invalid dual frame joint count");
      const bool now_attached = frames[f].at("box_attached");
      if (now_attached != attached) {
        require(q == prev, "dual attachment/release teleported");
        for (size_t j = 0; j < names.size(); ++j) state.setVariablePosition(names[j], q[j]);
        state.update(true);
        if (now_attached) {
          require(frames[f].at("stage") == "dual_3", "unexpected dual attachment");
          for (const auto& payload : payloads) {
            require((state.getGlobalLinkTransform(payload.tool) * payload.offset).translation().isApprox(payload.center, 1e-5),
              "dual payload not at source during attachment");
            scene.getWorldNonConst()->removeObject(payload.object);
            state.attachBody(payload.object, Eigen::Isometry3d::Identity(),
              {shapes::ShapeConstPtr(new shapes::Box(size.x(), size.y(), size.z()))},
              EigenSTL::vector_Isometry3d{payload.offset},
              std::set<std::string>{payload.tool, payload.side + "_joint7"}, payload.tool);
          }
          ++attachments;
        } else {
          require(frames[f].at("stage") == "dual_7", "unexpected dual release");
          for (const auto& payload : payloads) {
            const Eigen::Isometry3d box = state.getGlobalLinkTransform(payload.tool) * payload.offset;
            for (int i = 0; i < 8; ++i) {
              const Eigen::Vector3d corner((i & 1) ? size.x()/2 : -size.x()/2,
                (i & 2) ? size.y()/2 : -size.y()/2, (i & 4) ? size.z()/2 : -size.z()/2);
              require((box * corner).x() <= task.at("chassis_rear_x").get<double>() - .01 + 1e-8,
                "dual box not fully behind chassis at release");
            }
            state.clearAttachedBody(payload.object);
          }
          ++releases;
        }
        attached = now_attached;
      }
      size_t steps = 1;
      for (size_t j = 0; j < names.size(); ++j)
        steps = std::max(steps, static_cast<size_t>(std::ceil(std::abs(q[j]-prev[j]) /
          (names[j] == "updown" ? .0025 : .5 * std::acos(-1.) / 180.))));
      for (size_t step = 1; step <= steps; ++step) {
        for (size_t j = 0; j < names.size(); ++j)
          state.setVariablePosition(names[j], prev[j] + (q[j]-prev[j])*step/steps);
        state.update(true); require(state.satisfiesBounds(), "dual joint limit at frame " + std::to_string(f));
        auto acm = scene.getAllowedCollisionMatrix();
        if (!attached) for (const auto& payload : payloads)
          if ((state.getGlobalLinkTransform(payload.tool) * payload.offset).translation().isApprox(payload.center, 1e-5)) {
            acm.setEntry(payload.object, payload.tool, true);
            acm.setEntry(payload.object, payload.side + "_joint7", true);
          }
        collision_detection::CollisionRequest request; request.contacts = true; request.max_contacts = 10;
        collision_detection::CollisionResult result; scene.checkCollision(request, result, state, acm); ++checks;
        if (result.collision) {
          std::string pairs; for (const auto& contact : result.contacts)
            pairs += contact.first.first + "<->" + contact.first.second + " ";
          throw std::runtime_error("dual collision frame=" + std::to_string(f) + " " + pairs);
        }
      }
    }
    require(attachments == 1 && releases == 1 && !attached, "wrong dual payload lifecycle");
    std::cout << "PASS independent dual replay: " << frames.size() << " frames, " << checks << " probes\n";
}

int main(int argc, char** argv)
{
  try {
    require(argc == 4, "usage: validate_v3_wall_replay result.json robot.urdf robot.srdf");
    const auto task = Json::parse(read(argv[1]));
    auto urdf = urdf::parseURDF(read(argv[2]));
    auto srdf = std::make_shared<srdf::Model>();
    require(urdf && srdf->initString(*urdf, read(argv[3])), "invalid robot descriptions");
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    if (!task.value("sequence", false)) {
      validateCycle(task, model);
    } else {
      require(task.at("success") && task.at("completed_count") == 25, "requires all 25 successful cycles");
      Json previous = task.at("initial_joints");
      std::set<int> removed;
      size_t end = 0;
      for (const auto& box : task.at("boxes")) {
        Json cycle = box;
        const size_t begin = box.at("frame_begin"), next = box.at("frame_end");
        require(begin == end && next > begin, "non-contiguous cycle range");
        require(box.at("initial_joints") == previous, "cycle boundary teleported");
        require(box.at("removed_box_ids").get<std::set<int>>() == removed, "wrong prior boxes removed");
        require(box.at("neighbor_centers").size() == 24 - removed.size(), "missing neighbors");
        cycle["frames"] = Json::array();
        for (size_t f = begin; f < next; ++f) cycle["frames"].push_back(task.at("frames").at(f));
        const auto ids = box.contains("box_ids") ? box.at("box_ids").get<std::vector<int>>() :
          std::vector<int>{box.at("box_id").get<int>()};
        require(ids.size() == (box.value("dual", false) ? 2u : 1u), "invalid transfer box_ids");
        if (ids.size() == 2) validateDualCycle(cycle, model); else validateCycle(cycle, model);
        previous = cycle.at("frames").back().at("joints");
        removed.insert(ids.begin(), ids.end());
        end = next;
      }
      require(removed.size() == 25 && end == task.at("frames").size(), "incomplete sequence");
      require(previous == task.at("final_joints"), "wrong final posture");
      std::cout << "PASS all 25 boxes, unchanged-joint handoff and exact prior-box removal\n";
    }
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
