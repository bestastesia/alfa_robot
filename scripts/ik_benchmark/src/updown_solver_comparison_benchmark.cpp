#include "ik_benchmark/ik_solver.h"
#include "ik_benchmark/parallel_updown_aware_ik_solver.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using ik_benchmark::IkResult;
using ik_benchmark::IkSolver;
using ik_benchmark::IkSolverOptions;
using ik_benchmark::ParallelUpdownAwareIkSolver;
using ik_benchmark::UpdownAwareIkConfig;
using ik_benchmark::UpdownAwareIkRequest;

namespace {

struct PoseSpec {
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
};

struct PickPoint { PoseSpec left; PoseSpec right; };
struct Stage { std::string name; PoseSpec left; PoseSpec right; };

struct ComparisonConfig {
    double approach_offset = 0.1;
    double place_safe_z = 0.85;
    size_t unlimited_seed_attempts = 12;
    double unlimited_seed_noise = 0.35;
    size_t lookup_seed_count = 8;
    size_t lookup_h_candidate_count = 15;
    size_t lookup_workers = 1;
    size_t lookup_fallback_seed_count = 12;
};

struct LeverParams {
    double left_joint2_horizontal_angle = 0.0;
    double left_joint3_horizontal_angle = 0.0;
    double right_joint2_horizontal_angle = 0.0;
    double right_joint3_horizontal_angle = 0.0;
    double link2_length = 0.65;
    double link3_length = 0.65;
};

struct HeightInterval {
    bool reachable = false;
    double lower = 0.0;
    double upper = 0.0;
};

struct LookupHeightPlan {
    bool reachable = false;
    HeightInterval left;
    HeightInterval right;
    HeightInterval combined;
    std::vector<double> candidates;
};

struct LookupSolution {
    bool success = false;
    double h = 0.0;
    size_t h_index = 0;
    size_t seed_index = 0;
    double score = std::numeric_limits<double>::infinity();
    double solve_ms = 0.0;
    double direct_pos_error = 0.0;
    double direct_ori_error = 0.0;
    bool collision_free = false;
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    std::vector<std::string> full_joint_names;
    std::vector<double> full_joint_values;
    std::vector<std::string> collision_pairs;
};

LeverParams leverParamsFromConfig(const UpdownAwareIkConfig& config)
{
    return {
        config.left_joint2_horizontal_angle,
        config.left_joint3_horizontal_angle,
        config.right_joint2_horizontal_angle,
        config.right_joint3_horizontal_angle,
        config.link2_length,
        config.link3_length,
    };
}

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string stripInlineComment(std::string value)
{
    const auto pos = value.find('#');
    if (pos != std::string::npos) value = value.substr(0, pos);
    return trim(value);
}

bool parseBool(const std::string& value)
{
    return value == "true" || value == "True" || value == "1" || value == "yes";
}

double parseDouble(const std::string& value)
{
    const std::string normalized = trim(value);
    if (normalized == "inf" || normalized == "+inf" || normalized == ".inf") {
        return std::numeric_limits<double>::infinity();
    }
    return std::stod(normalized);
}

std::vector<double> parseDoubleList(std::string value)
{
    value = stripInlineComment(value);
    if (!value.empty() && value.front() == '[') value.erase(value.begin());
    if (!value.empty() && value.back() == ']') value.pop_back();
    std::vector<double> result;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = trim(token);
        if (!token.empty()) result.push_back(parseDouble(token));
    }
    return result;
}

std::vector<std::vector<double>> parsePoseFamilyDeg(std::string value)
{
    value = stripInlineComment(value);
    std::vector<std::vector<double>> family;
    std::stringstream stream(value);
    std::string segment;
    while (std::getline(stream, segment, ';')) {
        auto pose_deg = parseDoubleList(segment);
        if (pose_deg.size() != 6) {
            continue;
        }
        for (double& joint : pose_deg) {
            joint *= M_PI / 180.0;
        }
        family.push_back(std::move(pose_deg));
    }
    return family;
}

void setReachSphere(ik_benchmark::ReachSphereConfig& sphere, const std::vector<double>& values)
{
    if (values.size() >= 3) {
        sphere.cx = values[0];
        sphere.cy = values[1];
        sphere.cz = values[2];
    }
}

bool applyYamlValue(UpdownAwareIkConfig& config,
                    const std::vector<std::string>& path,
                    const std::string& key,
                    const std::string& raw_value)
{
    if (raw_value.empty()) return false;
    const std::string value = stripInlineComment(raw_value);
    const auto in = [&](std::initializer_list<const char*> expected) {
        if (path.size() != expected.size()) return false;
        size_t i = 0;
        for (const char* item : expected) {
            if (path[i++] != item) return false;
        }
        return true;
    };

    if (in({"groups"})) {
        if (key == "fixed_h_group") config.fixed_group = value;
        else if (key == "free_h_group") config.free_group = value;
        else if (key == "left_tip") config.left_tip = value;
        else if (key == "right_tip") config.right_tip = value;
        else if (key == "base_frame") config.base_frame = value;
        else if (key == "solver_plugin") config.solver_plugin = value;
        else return false;
        return true;
    }
    if (in({"h_planner", "left_reach_sphere"})) {
        if (key == "center") setReachSphere(config.left_reach_sphere, parseDoubleList(value));
        else if (key == "radius") config.left_reach_sphere.radius = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"h_planner", "right_reach_sphere"})) {
        if (key == "center") setReachSphere(config.right_reach_sphere, parseDoubleList(value));
        else if (key == "radius") config.right_reach_sphere.radius = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"h_planner"})) {
        if (key == "physical_limits") {
            const auto limits = parseDoubleList(value);
            if (limits.size() >= 2) {
                config.h_lower = limits[0];
                config.h_upper = limits[1];
            }
        } else if (key == "tool0_offset") config.tool0_offset = parseDouble(value);
        else if (key == "sphere_margin") config.sphere_margin = parseDouble(value);
        else if (key == "gripper_z_reach_window") {
            const auto window = parseDoubleList(value);
            if (window.size() >= 2) {
                config.gripper_z_reach_lower = window[0];
                config.gripper_z_reach_upper = window[1];
            }
        } else if (key == "top_suction_z_reach_window") {
            const auto window = parseDoubleList(value);
            if (window.size() >= 2) {
                config.top_suction_z_reach_lower = window[0];
                config.top_suction_z_reach_upper = window[1];
            }
        }
        else return false;
        return true;
    }
    if (in({"candidate_generator"})) {
        if (key == "h_mode") config.h_search_mode = value == "continuous_range" ? UpdownAwareIkConfig::HSearchMode::ContinuousRange : UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
        else if (key == "h_search_margin") config.h_search_margin = parseDouble(value);
        else if (key == "h_step") config.h_step = parseDouble(value);
        else if (key == "h_candidate_count") config.h_candidate_count = static_cast<size_t>(std::stoul(value));
        else if (key == "max_updown_delta") config.max_updown_delta = parseDouble(value);
        else if (key == "seed_count") config.seed_count = static_cast<size_t>(std::stoul(value));
        else if (key == "continuous_seed_multiplier") config.continuous_seed_multiplier = static_cast<size_t>(std::stoul(value));
        else if (key == "seed_noise") config.seed_noise = parseDouble(value);
        else if (key == "try_target_orders") config.try_target_orders = parseBool(value);
        else if (key == "use_reversed_target_order") config.use_reversed_target_order = parseBool(value);
        else return false;
        return true;
    }
    if (in({"parallel_executor"})) {
        if (key == "workers") config.workers = static_cast<size_t>(std::stoul(value));
        else if (key == "timeout_per_trial") config.timeout = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"validator"})) {
        if (key == "check_tip_error") config.check_tip_error = parseBool(value);
        else if (key == "position_tolerance") config.position_tolerance = parseDouble(value);
        else if (key == "orientation_tolerance") config.orientation_tolerance = parseDouble(value);
        else if (key == "check_collision") config.check_collision = parseBool(value);
        else if (key == "enforce_arm_base_collisions") config.enforce_arm_base_collisions = parseBool(value);
        else if (key == "reject_swapped_tips") config.reject_swapped_tips = parseBool(value);
        else return false;
        return true;
    }
    if (in({"fallback_manager"})) {
        if (key == "enabled") config.fallback_enabled = parseBool(value);
        else if (key == "release_updown_timeout") config.fallback_timeout = parseDouble(value);
        else if (key == "release_updown_seed_count") config.fallback_seed_count = static_cast<size_t>(std::stoul(value));
        else if (key == "rounds") config.fallback_rounds = static_cast<size_t>(std::stoul(value));
        else if (key == "random_family_count") config.fallback_random_family_count = static_cast<size_t>(std::stoul(value));
        else if (key == "random_per_family") config.fallback_random_per_family = static_cast<size_t>(std::stoul(value));
        else if (key == "seed_noise") config.fallback_seed_noise = parseDouble(value);
        else if (key == "updown_noise") config.fallback_updown_noise = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"cost_scorer", "weights"})) {
        if (key == "updown_static_bonus") config.cost_updown_static_bonus = parseDouble(value);
        else if (key == "updown_within_0p1_bonus") config.cost_updown_within_0p1_bonus = parseDouble(value);
        else if (key == "updown_over_0p1_distance") config.cost_updown_over_0p1_distance = parseDouble(value);
        else if (key == "joint2_torque") config.cost_joint2_torque = parseDouble(value);
        else if (key == "joint3_torque") config.cost_joint3_torque = parseDouble(value);
        else if (key == "loaded_family_distance") config.cost_loaded_family_distance = parseDouble(value);
        else if (key == "loaded_preferred_distance") config.cost_loaded_preferred_distance = parseDouble(value);
        else if (key == "solve_ms") config.cost_solve_ms = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"cost_scorer", "thresholds"})) {
        if (key == "updown_static_epsilon") config.updown_static_epsilon = parseDouble(value);
        else if (key == "updown_small_motion") config.updown_small_motion_threshold = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"cost_scorer", "torque_proxy"})) {
        if (key == "left_joint2_horizontal_angle") config.left_joint2_horizontal_angle = parseDouble(value);
        else if (key == "left_joint3_horizontal_angle") config.left_joint3_horizontal_angle = parseDouble(value);
        else if (key == "right_joint2_horizontal_angle") config.right_joint2_horizontal_angle = parseDouble(value);
        else if (key == "right_joint3_horizontal_angle") config.right_joint3_horizontal_angle = parseDouble(value);
        else if (key == "link2_length") config.link2_length = parseDouble(value);
        else if (key == "link3_length") config.link3_length = parseDouble(value);
        else if (key == "link2_mass_proxy") config.link2_mass_proxy = parseDouble(value);
        else if (key == "link3_mass_proxy") config.link3_mass_proxy = parseDouble(value);
        else if (key == "payload_mass_proxy") config.payload_mass_proxy = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"cost_scorer", "loaded_pose_prior"})) {
        if (key == "left_candidates_deg") config.left_loaded_pose_family = parsePoseFamilyDeg(value);
        else if (key == "right_candidates_deg") config.right_loaded_pose_family = parsePoseFamilyDeg(value);
        else if (key == "preferred_index") {
            config.left_preferred_loaded_pose_index = static_cast<size_t>(std::stoul(value));
            config.right_preferred_loaded_pose_index = static_cast<size_t>(std::stoul(value));
        }
        else if (key == "left_preferred_index") config.left_preferred_loaded_pose_index = static_cast<size_t>(std::stoul(value));
        else if (key == "right_preferred_index") config.right_preferred_loaded_pose_index = static_cast<size_t>(std::stoul(value));
        else return false;
        return true;
    }
    return false;
}

bool applyYamlValue(ComparisonConfig& config,
                    const std::vector<std::string>& path,
                    const std::string& key,
                    const std::string& raw_value)
{
    if (raw_value.empty()) return false;
    const std::string value = stripInlineComment(raw_value);
    const auto in = [&](std::initializer_list<const char*> expected) {
        if (path.size() != expected.size()) return false;
        size_t i = 0;
        for (const char* item : expected) {
            if (path[i++] != item) return false;
        }
        return true;
    };

    if (in({"benchmark_comparison", "flow"})) {
        if (key == "approach_offset") config.approach_offset = parseDouble(value);
        else if (key == "place_safe_z") config.place_safe_z = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"benchmark_comparison", "unlimited_baseline"})) {
        if (key == "seed_attempts") config.unlimited_seed_attempts = static_cast<size_t>(std::stoul(value));
        else if (key == "seed_noise") config.unlimited_seed_noise = parseDouble(value);
        else return false;
        return true;
    }
    if (in({"benchmark_comparison", "lookup_like"})) {
        if (key == "seed_count") config.lookup_seed_count = static_cast<size_t>(std::stoul(value));
        else if (key == "h_candidate_count") config.lookup_h_candidate_count = static_cast<size_t>(std::stoul(value));
        else if (key == "workers") config.lookup_workers = static_cast<size_t>(std::stoul(value));
        else if (key == "fallback_seed_count") config.lookup_fallback_seed_count = static_cast<size_t>(std::stoul(value));
        else return false;
        return true;
    }
    return false;
}

struct LoadedYamlConfig {
    UpdownAwareIkConfig ik;
    ComparisonConfig comparison;
};

LoadedYamlConfig loadConfigFromYaml(const std::filesystem::path& path)
{
    LoadedYamlConfig config;
    std::ifstream input(path);
    if (!input.good()) {
        throw std::runtime_error("Cannot read config: " + path.string());
    }

    std::vector<std::string> section_stack;
    std::string line;
    while (std::getline(input, line)) {
        if (trim(line).empty() || trim(line).front() == '#') continue;
        const size_t indent = line.find_first_not_of(' ');
        const size_t level = indent == std::string::npos ? 0 : indent / 2;
        std::string content = trim(line);
        content = stripInlineComment(content);
        if (content.empty()) continue;
        const auto colon = content.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(content.substr(0, colon));
        std::string value = trim(content.substr(colon + 1));
        if (level == 0 && key == "parallel_updown_aware_ik") {
            section_stack.clear();
            continue;
        }
        const size_t effective_level = level > 0 ? level - 1 : 0;
        if (value.empty()) {
            if (section_stack.size() < effective_level) section_stack.resize(effective_level);
            if (section_stack.size() == effective_level) section_stack.push_back(key);
            else section_stack[effective_level] = key;
            section_stack.resize(effective_level + 1);
            continue;
        }
        std::vector<std::string> path_stack = section_stack;
        if (path_stack.size() > effective_level) path_stack.resize(effective_level);
        applyYamlValue(config.ik, path_stack, key, value);
        applyYamlValue(config.comparison, path_stack, key, value);
    }
    return config;
}

Eigen::Isometry3d toIsometry(const PoseSpec& pose)
{
    Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    tf.linear() = q.toRotationMatrix();
    return tf;
}

nlohmann::json poseJson(const PoseSpec& pose)
{
    return {{"position", {pose.x, pose.y, pose.z}}, {"orientation", {pose.qx, pose.qy, pose.qz, pose.qw}}};
}

std::vector<PickPoint> pickPoints()
{
    const double qx = 0.0, qy = 0.7071, qz = 0.0, qw = 0.7071;
    return {
        {{0.6, 0.5, 1.5, qx, qy, qz, qw}, {0.6, -0.5, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.3, 1.5, qx, qy, qz, qw}, {0.6, -0.3, 1.5, qx, qy, qz, qw}},
        {{0.6, 0.0, 1.5, qx, qy, qz, qw}, {0.6, 0.0, 1.1, qx, qy, qz, qw}},
    };
}

std::vector<Stage> makeStages(size_t round, const PickPoint& point, double approach_offset, double place_safe_z)
{
    const std::string prefix = "round_" + std::to_string(round + 1);
    return {
        {prefix + "/safe",
         {0.3, 0.3, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {0.3, -0.3, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/approach",
         {point.left.x - approach_offset, point.left.y, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {point.right.x - approach_offset, point.right.y, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/grasp", point.left, point.right},
        {prefix + "/retreat",
         {point.left.x - approach_offset, point.left.y, point.left.z, point.left.qx, point.left.qy, point.left.qz, point.left.qw},
         {point.right.x - approach_offset, point.right.y, point.right.z, point.right.qx, point.right.qy, point.right.qz, point.right.qw}},
        {prefix + "/place_safe",
         {0.6, 0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5},
         {0.6, -0.2, place_safe_z, -0.5, 0.5, 0.5, 0.5}},
    };
}

Eigen::Isometry3d compensateTool0(const PoseSpec& pose, double tool0_offset)
{
    return toIsometry(pose) * Eigen::Translation3d(0.0, 0.0, -tool0_offset);
}

HeightInterval lookupIntervalForTarget(const PoseSpec& target,
                                       const ik_benchmark::ReachSphereConfig& sphere,
                                       double tool0_offset,
                                       double h_lower,
                                       double h_upper,
                                       double margin)
{
    const double radius = std::max(0.0, sphere.radius - margin);
    const double dx = target.x - sphere.cx;
    const double dy = target.y - sphere.cy;
    const double dxy2 = dx * dx + dy * dy;
    const double r2 = radius * radius;
    HeightInterval interval;
    if (dxy2 > r2) return interval;
    const double z_margin = std::sqrt(std::max(0.0, r2 - dxy2));
    const double ik_target_z = target.z - tool0_offset;
    interval.lower = std::max(h_lower, ik_target_z - sphere.cz - z_margin);
    interval.upper = std::min(h_upper, ik_target_z - sphere.cz + z_margin);
    interval.reachable = interval.lower <= interval.upper;
    return interval;
}

std::vector<double> lookupHeightCandidates(const HeightInterval& interval,
                                           double current_h,
                                           double step,
                                           size_t max_candidates)
{
    std::vector<double> candidates;
    if (!interval.reachable || max_candidates == 0) return candidates;
    auto add = [&](double value) {
        if (candidates.size() >= max_candidates) return;
        const double clamped = std::min(std::max(value, interval.lower), interval.upper);
        for (double existing : candidates) {
            if (std::abs(existing - clamped) < 1e-9) return;
        }
        candidates.push_back(clamped);
    };
    add(current_h);
    if (step <= 0.0) return candidates;
    for (size_t ring = 1; candidates.size() < max_candidates; ++ring) {
        const double delta = step * static_cast<double>(ring);
        bool added = false;
        if (current_h - delta >= interval.lower - 1e-9) {
            add(current_h - delta);
            added = true;
        }
        if (current_h + delta <= interval.upper + 1e-9) {
            add(current_h + delta);
            added = true;
        }
        if (!added && current_h - delta < interval.lower && current_h + delta > interval.upper) break;
    }
    return candidates;
}

LookupHeightPlan lookupPlanHeights(const Stage& stage,
                                   const UpdownAwareIkConfig& config,
                                   double current_h,
                                   size_t max_candidates)
{
    LookupHeightPlan plan;
    plan.left = lookupIntervalForTarget(stage.left, config.left_reach_sphere, config.tool0_offset,
                                        config.h_lower, config.h_upper, config.sphere_margin);
    plan.right = lookupIntervalForTarget(stage.right, config.right_reach_sphere, config.tool0_offset,
                                         config.h_lower, config.h_upper, config.sphere_margin);
    plan.combined.lower = std::max(plan.left.lower, plan.right.lower);
    plan.combined.upper = std::min(plan.left.upper, plan.right.upper);
    plan.combined.reachable = plan.left.reachable && plan.right.reachable && plan.combined.lower <= plan.combined.upper;
    plan.reachable = plan.combined.reachable;
    const double h_center = std::min(std::max(current_h, plan.combined.lower), plan.combined.upper);
    plan.candidates = lookupHeightCandidates(plan.combined, h_center, config.h_step, max_candidates);
    return plan;
}

std::vector<std::string> fullJointNamesForLookup(const std::vector<std::string>& arm_joint_names)
{
    std::vector<std::string> names;
    names.reserve(arm_joint_names.size() + 1);
    names.push_back("updown");
    names.insert(names.end(), arm_joint_names.begin(), arm_joint_names.end());
    return names;
}

std::vector<double> fullJointValuesForLookup(double h, const std::vector<double>& arm_values)
{
    std::vector<double> values;
    values.reserve(arm_values.size() + 1);
    values.push_back(h);
    values.insert(values.end(), arm_values.begin(), arm_values.end());
    return values;
}

double posError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    return (toIsometry(target).translation() - actual.translation()).norm();
}

double oriError(const PoseSpec& target, const Eigen::Isometry3d& actual)
{
    Eigen::AngleAxisd aa(toIsometry(target).linear().transpose() * actual.linear());
    return aa.angle();
}

double extractUpdown(const std::vector<std::string>& names, const std::vector<double>& values, double fallback)
{
    for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
        if (names[i] == "updown") return values[i];
    }
    return fallback;
}

double namedJointValue(const std::vector<std::string>& names,
                       const std::vector<double>& values,
                       const std::string& name,
                       double fallback = 0.0)
{
    for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
        if (names[i] == name) return values[i];
    }
    return fallback;
}

double angularDistance(double a, double b)
{
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

double loadedPoseDistance(const std::vector<std::string>& names,
                          const std::vector<double>& values,
                          const std::string& prefix,
                          const std::vector<double>& pose)
{
    if (pose.size() < 6) return 0.0;
    double squared_sum = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        const double joint = namedJointValue(names, values, prefix + "joint" + std::to_string(i + 1));
        const double diff = angularDistance(joint, pose[i]);
        squared_sum += diff * diff;
    }
    return std::sqrt(squared_sum);
}

double loadedPoseFamilyMinDistance(const std::vector<std::string>& names,
                                   const std::vector<double>& values,
                                   const std::string& prefix,
                                   const std::vector<std::vector<double>>& family)
{
    if (family.empty()) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pose : family) {
        if (pose.size() < 6) continue;
        best = std::min(best, loadedPoseDistance(names, values, prefix, pose));
    }
    return std::isfinite(best) ? best : 0.0;
}

double loadedPosePreferredDistance(const std::vector<std::string>& names,
                                   const std::vector<double>& values,
                                   const std::string& prefix,
                                   const std::vector<std::vector<double>>& family,
                                   size_t preferred_index)
{
    if (preferred_index >= family.size() || family[preferred_index].size() < 6) return 0.0;
    return loadedPoseDistance(names, values, prefix, family[preferred_index]);
}

double jointLeverProxy(const std::vector<std::string>& names,
                       const std::vector<double>& values,
                       const std::string& prefix,
                       int joint_index,
                       const LeverParams& params)
{
    const bool is_left = prefix == "left";
    const double q2 = namedJointValue(names, values, prefix + "joint2");
    const double q3 = namedJointValue(names, values, prefix + "joint3");
    const double q2_zero = is_left ? params.left_joint2_horizontal_angle : params.right_joint2_horizontal_angle;
    const double q3_zero = is_left ? params.left_joint3_horizontal_angle : params.right_joint3_horizontal_angle;
    const double shoulder_angle = q2 - q2_zero;
    const double elbow_angle = q2 + q3 - q2_zero - q3_zero;
    if (joint_index == 2) {
        return params.link2_length * std::abs(std::cos(shoulder_angle)) +
               params.link3_length * std::abs(std::cos(elbow_angle));
    }
    if (joint_index == 3) {
        return params.link3_length * std::abs(std::cos(elbow_angle));
    }
    return 0.0;
}

std::string hSearchModeName(UpdownAwareIkConfig::HSearchMode mode)
{
    return mode == UpdownAwareIkConfig::HSearchMode::ContinuousRange ? "continuous_range" : "fixed_discrete";
}

UpdownAwareIkConfig::HSearchMode parseHSearchMode(const std::string& value)
{
    if (value == "continuous_range" || value == "continuous") {
        return UpdownAwareIkConfig::HSearchMode::ContinuousRange;
    }
    if (value == "fixed_discrete" || value == "fixed") {
        return UpdownAwareIkConfig::HSearchMode::FixedDiscrete;
    }
    throw std::runtime_error("Unsupported --h-mode: " + value);
}

struct StrategySelection {
    bool unlimited = true;
    bool lookup = true;
    bool experiment = true;
};

StrategySelection parseStrategies(const std::string& raw)
{
    StrategySelection selection{false, false, false};
    std::stringstream stream(raw);
    std::string token;
    while (std::getline(stream, token, ',')) {
        token = trim(token);
        if (token.empty()) continue;
        if (token == "all") {
            selection = {true, true, true};
        } else if (token == "unlimited" || token == "baseline1") {
            selection.unlimited = true;
        } else if (token == "lookup" || token == "baseline2") {
            selection.lookup = true;
        } else if (token == "experiment" || token == "parallel" || token == "new") {
            selection.experiment = true;
        } else {
            throw std::runtime_error("Unsupported --strategy value: " + token);
        }
    }
    return selection;
}

nlohmann::json enabledStrategiesJson(const StrategySelection& selection)
{
    nlohmann::json strategies = nlohmann::json::array();
    if (selection.unlimited) strategies.push_back("unlimited_bioik_until_collision_free");
    if (selection.lookup) strategies.push_back("lookup_like_fixed_h_with_fallback");
    if (selection.experiment) strategies.push_back("parallel_updown_aware_solver");
    return strategies;
}

nlohmann::json costWeightsJson(const UpdownAwareIkConfig& config)
{
    return {
        {"updown_static_bonus", config.cost_updown_static_bonus},
        {"updown_within_0p1_bonus", config.cost_updown_within_0p1_bonus},
        {"updown_over_0p1_distance", config.cost_updown_over_0p1_distance},
        {"joint2_torque", config.cost_joint2_torque},
        {"joint3_torque", config.cost_joint3_torque},
        {"loaded_family_distance", config.cost_loaded_family_distance},
        {"loaded_preferred_distance", config.cost_loaded_preferred_distance},
        {"solve_ms", config.cost_solve_ms},
    };
}

nlohmann::json costThresholdsJson(const UpdownAwareIkConfig& config)
{
    return {
        {"updown_static_epsilon", config.updown_static_epsilon},
        {"updown_small_motion", config.updown_small_motion_threshold},
    };
}

struct TorqueProxyBreakdown {
    double joint2 = 0.0;
    double joint3 = 0.0;
};

TorqueProxyBreakdown torqueProxyBreakdown(const std::vector<std::string>& names,
                                          const std::vector<double>& values,
                                          const std::string& prefix,
                                          const UpdownAwareIkConfig& config)
{
    const bool is_left = prefix == "left";
    const double q2 = namedJointValue(names, values, prefix + "joint2");
    const double q3 = namedJointValue(names, values, prefix + "joint3");
    const double q2_zero = is_left ? config.left_joint2_horizontal_angle : config.right_joint2_horizontal_angle;
    const double q3_zero = is_left ? config.left_joint3_horizontal_angle : config.right_joint3_horizontal_angle;
    const double shoulder_angle = q2 - q2_zero;
    const double elbow_angle = q2 + q3 - q2_zero - q3_zero;

    const double link2_moment = config.link2_mass_proxy * 0.5 * config.link2_length * std::abs(std::cos(shoulder_angle));
    const double link3_shoulder_moment = config.link3_mass_proxy *
        (config.link2_length * std::abs(std::cos(shoulder_angle)) +
         0.5 * config.link3_length * std::abs(std::cos(elbow_angle)));
    const double payload_shoulder_moment = config.payload_mass_proxy *
        (config.link2_length * std::abs(std::cos(shoulder_angle)) +
         config.link3_length * std::abs(std::cos(elbow_angle)));
    const double link3_elbow_moment = config.link3_mass_proxy * 0.5 * config.link3_length * std::abs(std::cos(elbow_angle));
    const double payload_elbow_moment = config.payload_mass_proxy * config.link3_length * std::abs(std::cos(elbow_angle));

    TorqueProxyBreakdown out;
    out.joint2 = config.cost_joint2_torque * (link2_moment + link3_shoulder_moment + payload_shoulder_moment);
    out.joint3 = config.cost_joint3_torque * (link3_elbow_moment + payload_elbow_moment);
    return out;
}


nlohmann::json candidateCostBreakdownJson(const ik_benchmark::UpdownAwareIkCandidate& candidate,
                                          const UpdownAwareIkConfig& config)
{
    const auto left = torqueProxyBreakdown(candidate.full_joint_names, candidate.full_joint_values, "left", config);
    const auto right = torqueProxyBreakdown(candidate.full_joint_names, candidate.full_joint_values, "right", config);

    nlohmann::json breakdown;
    breakdown["updown_static_bonus"] = candidate.updown_delta <= config.updown_static_epsilon ? -config.cost_updown_static_bonus : 0.0;
    breakdown["updown_small_motion_bonus"] = candidate.updown_delta <= config.updown_small_motion_threshold ? -config.cost_updown_within_0p1_bonus : 0.0;
    breakdown["updown_over_small_motion_penalty"] = candidate.updown_delta > config.updown_small_motion_threshold
        ? config.cost_updown_over_0p1_distance * (candidate.updown_delta - config.updown_small_motion_threshold)
        : 0.0;
    breakdown["left_joint2_torque"] = left.joint2;
    breakdown["right_joint2_torque"] = right.joint2;
    breakdown["left_joint3_torque"] = left.joint3;
    breakdown["right_joint3_torque"] = right.joint3;
    const double left_family_distance = loadedPoseFamilyMinDistance(
        candidate.full_joint_names, candidate.full_joint_values, "left", config.left_loaded_pose_family);
    const double right_family_distance = loadedPoseFamilyMinDistance(
        candidate.full_joint_names, candidate.full_joint_values, "right", config.right_loaded_pose_family);
    const double left_preferred_distance = loadedPosePreferredDistance(
        candidate.full_joint_names, candidate.full_joint_values, "left", config.left_loaded_pose_family,
        config.left_preferred_loaded_pose_index);
    const double right_preferred_distance = loadedPosePreferredDistance(
        candidate.full_joint_names, candidate.full_joint_values, "right", config.right_loaded_pose_family,
        config.right_preferred_loaded_pose_index);
    breakdown["left_loaded_family_distance"] = config.cost_loaded_family_distance * left_family_distance;
    breakdown["right_loaded_family_distance"] = config.cost_loaded_family_distance * right_family_distance;
    breakdown["left_loaded_preferred_distance"] = config.cost_loaded_preferred_distance * left_preferred_distance;
    breakdown["right_loaded_preferred_distance"] = config.cost_loaded_preferred_distance * right_preferred_distance;
    breakdown["solve_ms"] = config.cost_solve_ms * candidate.solve_ms;
    breakdown["total"] = candidate.score;
    return breakdown;
}

nlohmann::json serializeCandidate(const ik_benchmark::UpdownAwareIkCandidate& candidate,
                                  size_t candidate_index,
                                  size_t rank,
                                  bool selected,
                                  const LeverParams& lever_params,
                                  const UpdownAwareIkConfig& config)
{
    nlohmann::json out;
    out["candidate_index"] = candidate_index;
    if (candidate.legal) out["rank"] = rank;
    else out["rank"] = nullptr;
    out["selected"] = selected;
    out["legal"] = candidate.legal;
    out["collision_free"] = candidate.collision_free;
    out["swapped"] = candidate.swapped;
    out["timeout_like"] = candidate.timeout_like;
    out["solver_path"] = candidate.solver_path;
    out["target_order"] = candidate.target_order;
    out["rejection_reason"] = candidate.rejection_reason;
    out["h"] = candidate.h;
    out["h_center"] = candidate.h_center;
    out["h_range_lower"] = candidate.h_range_lower;
    out["h_range_upper"] = candidate.h_range_upper;
    out["h_index"] = candidate.h_index;
    out["seed_index"] = candidate.seed_index;
    out["score"] = candidate.score;
    out["solve_ms"] = candidate.solve_ms;
    out["direct_pos_error"] = candidate.direct_pos_error;
    out["direct_ori_error"] = candidate.direct_ori_error;
    out["swapped_pos_error"] = candidate.swapped_pos_error;
    out["updown_delta"] = candidate.updown_delta;
    out["joint_delta"] = candidate.joint_delta;
    out["joint_names"] = candidate.joint_names;
    out["joint_values"] = candidate.joint_values;
    out["full_joint_names"] = candidate.full_joint_names;
    out["full_joint_values"] = candidate.full_joint_values;
    out["collision_pairs"] = candidate.collision_pairs;

    const double left_j2 = jointLeverProxy(candidate.full_joint_names, candidate.full_joint_values, "left", 2, lever_params);
    const double right_j2 = jointLeverProxy(candidate.full_joint_names, candidate.full_joint_values, "right", 2, lever_params);
    const double left_j3 = jointLeverProxy(candidate.full_joint_names, candidate.full_joint_values, "left", 3, lever_params);
    const double right_j3 = jointLeverProxy(candidate.full_joint_names, candidate.full_joint_values, "right", 3, lever_params);
    out["left_joint2_lever_length"] = left_j2;
    out["right_joint2_lever_length"] = right_j2;
    out["joint2_lever_length"] = std::max(left_j2, right_j2);
    out["left_joint3_lever_length"] = left_j3;
    out["right_joint3_lever_length"] = right_j3;
    out["joint3_lever_length"] = std::max(left_j3, right_j3);
    out["cost_breakdown"] = candidateCostBreakdownJson(candidate, config);
    return out;
}

std::vector<size_t> legalRankOrder(const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates)
{
    std::vector<size_t> order;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].legal) order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return candidates[a].score < candidates[b].score;
    });
    return order;
}

bool sameSelectedCandidate(const ik_benchmark::UpdownAwareIkCandidate& a,
                           const ik_benchmark::UpdownAwareIkCandidate& b)
{
    return a.legal == b.legal &&
           a.h_index == b.h_index &&
           a.seed_index == b.seed_index &&
           a.target_order == b.target_order &&
           a.solver_path == b.solver_path &&
           std::abs(a.h - b.h) < 1e-9 &&
           std::abs(a.score - b.score) < 1e-9;
}

nlohmann::json serializeCandidates(const std::vector<ik_benchmark::UpdownAwareIkCandidate>& candidates,
                                   const ik_benchmark::UpdownAwareIkCandidate& selected,
                                   const LeverParams& lever_params,
                                   const UpdownAwareIkConfig& config,
                                   size_t& selected_rank,
                                   size_t& selected_index)
{
    const auto order = legalRankOrder(candidates);
    std::map<size_t, size_t> rank_by_index;
    for (size_t i = 0; i < order.size(); ++i) {
        rank_by_index[order[i]] = i + 1;
    }

    nlohmann::json out = nlohmann::json::array();
    selected_rank = 0;
    selected_index = candidates.size();
    for (size_t i = 0; i < candidates.size(); ++i) {
        const bool selected_candidate = selected.legal && sameSelectedCandidate(candidates[i], selected);
        const size_t rank = rank_by_index.count(i) ? rank_by_index[i] : 0;
        if (selected_candidate) {
            selected_rank = rank;
            selected_index = i;
        }
        out.push_back(serializeCandidate(candidates[i], i, rank, selected_candidate, lever_params, config));
    }
    return out;
}

void addSelectedJointDiagnostics(nlohmann::json& record,
                                 const std::vector<std::string>& joint_names,
                                 const std::vector<double>& joint_values,
                                 const LeverParams& lever_params)
{
    record["selected_joint_names"] = joint_names;
    record["selected_joint_values"] = joint_values;
    const double left_j2 = jointLeverProxy(joint_names, joint_values, "left", 2, lever_params);
    const double right_j2 = jointLeverProxy(joint_names, joint_values, "right", 2, lever_params);
    const double left_j3 = jointLeverProxy(joint_names, joint_values, "left", 3, lever_params);
    const double right_j3 = jointLeverProxy(joint_names, joint_values, "right", 3, lever_params);
    record["left_joint2_lever_length"] = left_j2;
    record["right_joint2_lever_length"] = right_j2;
    record["joint2_lever_length"] = std::max(left_j2, right_j2);
    record["left_joint3_lever_length"] = left_j3;
    record["right_joint3_lever_length"] = right_j3;
    record["joint3_lever_length"] = std::max(left_j3, right_j3);
}

std::vector<double> perturbSeed(const std::vector<std::string>& names,
                                std::vector<double> seed,
                                size_t attempt,
                                double revolute_noise,
                                double h_noise,
                                double h_lower,
                                double h_upper)
{
    if (seed.size() != names.size()) seed.assign(names.size(), 0.0);
    std::mt19937 rng(static_cast<uint32_t>(0xBAD5EEDu + attempt * 7919u));
    std::normal_distribution<double> revolute(0.0, revolute_noise);
    std::normal_distribution<double> prismatic(0.0, h_noise);
    for (size_t i = 0; i < seed.size(); ++i) {
        if (names[i] == "updown") seed[i] = std::min(std::max(seed[i] + prismatic(rng), h_lower), h_upper);
        else if (names[i].find("pitch") == std::string::npos) seed[i] += revolute(rng);
    }
    return seed;
}

std::string ikAttemptKey(const std::string& solver_path,
                         const std::string& target_order,
                         double h_lower,
                         double h_upper,
                         const std::vector<std::string>& names,
                         const std::vector<double>& seed)
{
    std::ostringstream out;
    out.precision(17);
    out << solver_path << '|' << target_order << '|' << h_lower << '|' << h_upper << '|';
    for (size_t i = 0; i < names.size() && i < seed.size(); ++i) {
        out << names[i] << '=' << seed[i] << ';';
    }
    return out.str();
}

bool markUniqueAttempt(std::set<std::string>& seen,
                       const std::string& solver_path,
                       const std::string& target_order,
                       double h_lower,
                       double h_upper,
                       const std::vector<std::string>& names,
                       const std::vector<double>& seed)
{
    return seen.insert(ikAttemptKey(solver_path, target_order, h_lower, h_upper, names, seed)).second;
}

std::vector<double> armSeedFromFull(const std::vector<std::string>& arm_names,
                                    const IkResult& result)
{
    std::unordered_map<std::string, double> values;
    for (size_t i = 0; i < result.joint_names.size() && i < result.joint_values.size(); ++i) {
        values[result.joint_names[i]] = result.joint_values[i];
    }
    std::vector<double> seed(arm_names.size(), 0.0);
    for (size_t i = 0; i < arm_names.size(); ++i) {
        if (auto it = values.find(arm_names[i]); it != values.end()) seed[i] = it->second;
    }
    return seed;
}

bool tipBindingOk(const Stage& stage, const std::vector<Eigen::Isometry3d>& poses, double pos_tol, double ori_tol, nlohmann::json& record)
{
    if (poses.size() < 2) {
        record["rejection_reason"] = "fk_missing_dual_tip";
        return false;
    }
    const double direct_pos = std::max(posError(stage.left, poses[0]), posError(stage.right, poses[1]));
    const double swapped_pos = std::max(posError(stage.left, poses[1]), posError(stage.right, poses[0]));
    const double direct_ori = std::max(oriError(stage.left, poses[0]), oriError(stage.right, poses[1]));
    record["direct_pos_error"] = direct_pos;
    record["direct_ori_error"] = direct_ori;
    record["swapped_pos_error"] = swapped_pos;
    if (swapped_pos + 1e-4 < direct_pos) {
        record["rejection_reason"] = "tip_order_error";
        return false;
    }
    if (direct_pos > pos_tol || direct_ori > ori_tol) {
        record["rejection_reason"] = "tip_error_too_large";
        return false;
    }
    return true;
}

nlohmann::json runUnlimitedStage(IkSolver& ik,
                                 const Stage& stage,
                                 double current_h,
                                 std::vector<double>& full_seed,
                                 double timeout,
                                 size_t seed_attempts,
                                 double seed_noise,
                                 double h_step,
                                 double h_lower,
                                 double h_upper,
                                 double tool0_offset,
                                 double pos_tol,
                                 double ori_tol,
                                 const LeverParams& lever_params)
{
    nlohmann::json record;
    record["strategy"] = "unlimited_bioik_until_collision_free";
    record["current_h"] = current_h;
    record["attempts"] = nlohmann::json::array();
    bool success = false;
    IkResult selected;
    double selected_h = current_h;
    std::vector<double> selected_seed = full_seed;
    std::set<std::string> seen_attempts;

    for (size_t attempt = 0; attempt < seed_attempts && !success; ++attempt) {
        auto seed = attempt == 0 ? full_seed : perturbSeed(ik.variableNames(), full_seed, attempt, seed_noise, h_step, h_lower, h_upper);
        for (size_t order = 0; order < 2 && !success; ++order) {
            const bool swapped_order = order == 1;
            if (!markUniqueAttempt(seen_attempts, "unlimited_bioik", swapped_order ? "swapped" : "normal",
                                   h_lower, h_upper, ik.variableNames(), seed)) {
                continue;
            }
            IkResult result = swapped_order
                ? ik.solveDual(compensateTool0(stage.right, tool0_offset), compensateTool0(stage.left, tool0_offset), seed, timeout)
                : ik.solveDual(compensateTool0(stage.left, tool0_offset), compensateTool0(stage.right, tool0_offset), seed, timeout);
            nlohmann::json attempt_json;
            attempt_json["attempt"] = attempt;
            attempt_json["target_order"] = swapped_order ? "swapped" : "normal";
            attempt_json["success_raw"] = result.success;
            attempt_json["solve_ms"] = result.solve_ms;
            attempt_json["timeout_like"] = result.solve_ms >= timeout * 1000.0 * 0.9;
            attempt_json["joint_names"] = result.joint_names;
            bool ok = false;
            if (!result.joint_values.empty()) {
                auto poses = ik.fk(result.joint_values);
                ok = tipBindingOk(stage, poses, pos_tol, ori_tol, attempt_json);
                std::vector<std::string> collision_pairs;
                const bool collision_free = ik.isNamedStateCollisionFree(result.joint_names, result.joint_values, &collision_pairs);
                attempt_json["collision_free"] = collision_free;
                attempt_json["collision_pairs"] = collision_pairs;
                ok = ok && collision_free;
            } else {
                attempt_json["rejection_reason"] = "ik_failed_empty_solution";
            }
            record["attempts"].push_back(attempt_json);
            if (ok) {
                selected = result;
                selected_seed = seed;
                selected_h = extractUpdown(result.joint_names, result.joint_values, current_h);
                success = true;
            }
        }
    }

    record["success"] = success;
    record["selected_h"] = selected_h;
    record["updown_delta"] = std::abs(selected_h - current_h);
    record["attempt_count"] = record["attempts"].size();
    record["total_solve_ms"] = 0.0;
    for (const auto& a : record["attempts"]) record["total_solve_ms"] = record["total_solve_ms"].get<double>() + a.value("solve_ms", 0.0);
    if (success) {
        full_seed = selected_seed;
        addSelectedJointDiagnostics(record, selected.joint_names, selected.joint_values, lever_params);
    }
    return record;
}

nlohmann::json runLookupStage(IkSolver& arm_ik,
                              IkSolver& fallback_ik,
                              ParallelUpdownAwareIkSolver& solver,
                              const Stage& stage,
                              double current_h,
                              std::vector<double>& arm_seed,
                              std::vector<double>& fallback_seed,
                              size_t seed_attempts,
                              size_t fallback_attempts,
                              const LeverParams& lever_params,
                              const UpdownAwareIkConfig& config)
{
    const auto t0 = std::chrono::steady_clock::now();
    const auto plan = lookupPlanHeights(stage, config, current_h, config.h_candidate_count);
    const auto full_joint_names = fullJointNamesForLookup(arm_ik.variableNames());
    const size_t target_legal_count = std::max<size_t>(1, fallback_attempts == 0 ? 5 : std::min<size_t>(5, fallback_attempts));

    nlohmann::json record;
    record["strategy"] = "lookup_like_fixed_h_with_fallback";
    record["selection_policy"] = "legacy_lookup_collect_few_min_updown";
    record["current_h"] = current_h;
    record["h_interval"] = {{"lower", plan.combined.lower}, {"upper", plan.combined.upper}, {"reachable", plan.reachable}};
    record["h_candidates"] = plan.candidates;

    std::vector<LookupSolution> legal_solutions;
    size_t trial_count = 0;
    size_t timeout_like_count = 0;
    double sum_solve_ms = 0.0;
    bool fallback_used = false;
    std::set<std::string> seen_attempts;

    if (plan.reachable && !plan.candidates.empty()) {
        for (size_t h_index = 0; h_index < plan.candidates.size() && legal_solutions.size() < target_legal_count; ++h_index) {
            const double candidate_h = plan.candidates[h_index];
            Eigen::Isometry3d left_target = compensateTool0(stage.left, config.tool0_offset);
            Eigen::Isometry3d right_target = compensateTool0(stage.right, config.tool0_offset);
            left_target.translation().z() -= candidate_h;
            right_target.translation().z() -= candidate_h;

            for (size_t seed_index = 0; seed_index < std::max<size_t>(1, seed_attempts) && legal_solutions.size() < target_legal_count; ++seed_index) {
                const auto seed = seed_index == 0
                    ? arm_seed
                    : perturbSeed(arm_ik.variableNames(), arm_seed,
                                  seed_index + h_index * std::max<size_t>(1, seed_attempts),
                                  config.seed_noise, 0.0, config.h_lower, config.h_upper);

                if (!markUniqueAttempt(seen_attempts, "legacy_fixed_h", "normal", candidate_h, candidate_h, arm_ik.variableNames(), seed)) {
                    continue;
                }
                IkResult result = arm_ik.solveDual(left_target, right_target, seed, config.timeout);
                ++trial_count;
                sum_solve_ms += result.solve_ms;
                if (result.solve_ms >= config.timeout * 1000.0 * 0.9) ++timeout_like_count;

                if (result.joint_values.empty()) continue;
                const auto full_values = fullJointValuesForLookup(candidate_h, result.joint_values);
                nlohmann::json validation;
                const auto poses = arm_ik.fkNamed(full_joint_names, full_values);
                if (!tipBindingOk(stage, poses, config.position_tolerance, config.orientation_tolerance, validation)) continue;
                std::vector<std::string> collision_pairs;
                const bool collision_free = arm_ik.isNamedStateCollisionFree(full_joint_names, full_values, &collision_pairs);
                if (!collision_free) continue;

                LookupSolution solution;
                solution.success = true;
                solution.h = candidate_h;
                solution.h_index = h_index;
                solution.seed_index = seed_index;
                solution.solve_ms = result.solve_ms;
                solution.direct_pos_error = validation.value("direct_pos_error", 0.0);
                solution.direct_ori_error = validation.value("direct_ori_error", 0.0);
                solution.collision_free = true;
                solution.joint_names = result.joint_names;
                solution.joint_values = result.joint_values;
                solution.full_joint_names = full_joint_names;
                solution.full_joint_values = full_values;
                solution.collision_pairs = collision_pairs;
                solution.score = std::abs(candidate_h - current_h) + 0.001 * result.solve_ms + 0.01 * static_cast<double>(seed_index);
                legal_solutions.push_back(std::move(solution));
            }
        }
    }

    if (legal_solutions.empty()) {
        fallback_used = true;
        for (size_t seed_index = 0; seed_index < std::max<size_t>(1, fallback_attempts) && legal_solutions.empty(); ++seed_index) {
            const auto seed = seed_index == 0
                ? fallback_seed
                : perturbSeed(fallback_ik.variableNames(), fallback_seed,
                              100000 + seed_index, config.seed_noise,
                              std::max(config.h_step, config.h_search_margin), config.h_lower, config.h_upper);
            for (size_t order = 0; order < 2 && legal_solutions.empty(); ++order) {
                const bool swapped_order = order == 1;
                if (!markUniqueAttempt(seen_attempts, "legacy_release_updown_fallback", swapped_order ? "swapped" : "normal",
                                       config.h_lower, config.h_upper, fallback_ik.variableNames(), seed)) {
                    continue;
                }
                IkResult result = swapped_order
                    ? fallback_ik.solveDual(compensateTool0(stage.right, config.tool0_offset), compensateTool0(stage.left, config.tool0_offset), seed, config.fallback_timeout)
                    : fallback_ik.solveDual(compensateTool0(stage.left, config.tool0_offset), compensateTool0(stage.right, config.tool0_offset), seed, config.fallback_timeout);
                ++trial_count;
                sum_solve_ms += result.solve_ms;
                if (result.solve_ms >= config.fallback_timeout * 1000.0 * 0.9) ++timeout_like_count;
                if (result.joint_values.empty()) continue;
                nlohmann::json validation;
                const auto poses = fallback_ik.fk(result.joint_values);
                if (!tipBindingOk(stage, poses, config.position_tolerance, config.orientation_tolerance, validation)) continue;
                std::vector<std::string> collision_pairs;
                const bool collision_free = fallback_ik.isNamedStateCollisionFree(result.joint_names, result.joint_values, &collision_pairs);
                if (!collision_free) continue;
                LookupSolution solution;
                solution.success = true;
                solution.h = extractUpdown(result.joint_names, result.joint_values, current_h);
                solution.h_index = 0;
                solution.seed_index = seed_index;
                solution.solve_ms = result.solve_ms;
                solution.direct_pos_error = validation.value("direct_pos_error", 0.0);
                solution.direct_ori_error = validation.value("direct_ori_error", 0.0);
                solution.collision_free = true;
                solution.joint_names = result.joint_names;
                solution.joint_values = result.joint_values;
                solution.full_joint_names = result.joint_names;
                solution.full_joint_values = result.joint_values;
                solution.collision_pairs = collision_pairs;
                solution.score = std::abs(solution.h - current_h) + 0.001 * result.solve_ms + 0.01 * static_cast<double>(seed_index);
                legal_solutions.push_back(std::move(solution));
            }
        }
    }

    const bool success = !legal_solutions.empty();
    LookupSolution selected;
    if (success) {
        selected = *std::min_element(legal_solutions.begin(), legal_solutions.end(), [](const auto& a, const auto& b) {
            return a.score < b.score;
        });
        arm_seed = armSeedFromFull(arm_ik.variableNames(), {true, true, selected.collision_free, 0, selected.joint_names, selected.joint_values, selected.collision_pairs, selected.solve_ms, selected.direct_pos_error, selected.direct_ori_error});
        fallback_seed = selected.full_joint_values;
    }

    const auto t1 = std::chrono::steady_clock::now();
    record["success"] = success;
    record["fallback_used"] = fallback_used;
    record["solver_path"] = fallback_used ? "legacy_release_updown_fallback" : (success && std::abs(selected.h - current_h) < 1e-9 ? "fixed_current_h" : "fixed_h_candidates");
    record["selected_h"] = success ? selected.h : current_h;
    record["updown_delta"] = success ? std::abs(selected.h - current_h) : 0.0;
    record["trial_count"] = trial_count;
    record["legal_count"] = legal_solutions.size();
    record["timeout_like_count"] = timeout_like_count;
    record["wall_ms"] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    record["sum_solve_ms"] = sum_solve_ms;
    if (success) {
        record["selected_score"] = selected.score;
        record["lookup_selection_score"] = selected.score;
        record["selected_h_index"] = selected.h_index;
        record["selected_seed_index"] = selected.seed_index;
        record["direct_pos_error"] = selected.direct_pos_error;
        record["direct_ori_error"] = selected.direct_ori_error;
        record["collision_free"] = selected.collision_free;
        record["collision_pairs"] = selected.collision_pairs;
        addSelectedJointDiagnostics(record, selected.full_joint_names, selected.full_joint_values, lever_params);
    } else {
        record["failure_reason"] = plan.reachable ? "legacy_lookup_no_legal_solution" : "reachability_interval_empty";
    }
    (void)solver;
    return record;
}

nlohmann::json runNewSolverStage(ParallelUpdownAwareIkSolver& solver,
                                 const Stage& stage,
                                 double current_h,
                                 std::vector<double>& arm_seed,
                                 std::vector<double>& full_seed,
                                 const LeverParams& lever_params,
                                 const UpdownAwareIkConfig& config)
{
    UpdownAwareIkRequest request;
    request.left_target = toIsometry(stage.left);
    request.right_target = toIsometry(stage.right);
    request.current_h = current_h;
    request.current_arm_joints = arm_seed;
    request.current_full_joints = full_seed;
    auto result = solver.solve(request);

    size_t selected_rank = 0;
    size_t selected_index = result.candidates.size();
    nlohmann::json candidates = serializeCandidates(result.candidates, result.selected, lever_params, config,
                                                     selected_rank, selected_index);

    nlohmann::json record;
    record["schema_version"] = 2;
    record["strategy"] = "parallel_updown_aware_solver";
    record["success"] = result.success;
    record["fallback_used"] = result.fallback_used;
    record["solver_path"] = result.solver_path;
    record["current_h"] = current_h;
    record["selected_h"] = result.success ? result.selected.h : current_h;
    record["updown_delta"] = result.success ? std::abs(result.selected.h - current_h) : 0.0;
    record["h_interval"] = {{"lower", result.h_interval_lower}, {"upper", result.h_interval_upper}, {"reachable", result.range_reachable}};
    record["h_interval_lower"] = result.h_interval_lower;
    record["h_interval_upper"] = result.h_interval_upper;
    record["h_interval_reachable"] = result.range_reachable;
    record["h_candidates"] = result.h_candidates;
    record["h_candidate_count"] = result.h_candidates.size();
    record["trial_count"] = result.trial_count;
    record["legal_count"] = result.legal_count;
    record["candidate_count"] = result.candidates.size();
    record["legal_candidate_count"] = result.legal_count;
    record["rejected_candidate_count"] = result.candidates.size() >= result.legal_count ? result.candidates.size() - result.legal_count : 0;
    record["timeout_like_count"] = result.timeout_like_count;
    record["wall_ms"] = result.wall_ms;
    record["sum_solve_ms"] = result.sum_solve_ms;
    record["candidates"] = candidates;
    if (result.success) {
        arm_seed = result.selected.joint_values;
        full_seed = result.selected.full_joint_values;
        record["selected_score"] = result.selected.score;
        record["selected_candidate_index"] = selected_index;
        record["selected_rank"] = selected_rank;
        record["selected_h_index"] = result.selected.h_index;
        record["selected_seed_index"] = result.selected.seed_index;
        record["selected_target_order"] = result.selected.target_order;
        record["selected_solver_path"] = result.selected.solver_path;
        record["selected_joint_delta"] = result.selected.joint_delta;
        record["selected_timeout_like"] = result.selected.timeout_like;
        record["selected_cost_breakdown"] = candidateCostBreakdownJson(result.selected, config);
        record["direct_pos_error"] = result.selected.direct_pos_error;
        record["direct_ori_error"] = result.selected.direct_ori_error;
        record["collision_free"] = result.selected.collision_free;
        record["collision_pairs"] = result.selected.collision_pairs;
        addSelectedJointDiagnostics(record, result.selected.full_joint_names, result.selected.full_joint_values, lever_params);
    } else {
        record["failure_reason"] = result.failure_reason;
    }
    return record;
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path output = std::filesystem::path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/updown_solver_comparison.jsonl");
    std::filesystem::path config_path = std::filesystem::path("/mnt/mydisk/ALFA/alfa_robot/scripts/ik_benchmark/config/parallel_updown_aware_ik.yaml");
    size_t rounds = 3;
    size_t max_stages = 0;
    bool seed_attempts_override = false;
    size_t seed_attempts_value = 0;
    bool timeout_override = false;
    double timeout_value = 0.0;
    bool fallback_timeout_override = false;
    double fallback_timeout_value = 0.0;
    bool workers_override = false;
    size_t workers_value = 0;
    StrategySelection strategy_selection;
    bool h_mode_override = false;
    UpdownAwareIkConfig::HSearchMode h_mode_value = UpdownAwareIkConfig::HSearchMode::FixedDiscrete;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--output" || arg == "--jsonl") && i + 1 < argc) output = argv[++i];
        else if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (arg == "--rounds" && i + 1 < argc) rounds = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--timeout" && i + 1 < argc) {
            timeout_value = std::stod(argv[++i]);
            timeout_override = true;
        }
        else if (arg == "--seed-attempts" && i + 1 < argc) {
            seed_attempts_value = static_cast<size_t>(std::stoul(argv[++i]));
            seed_attempts_override = true;
        }
        else if (arg == "--workers" && i + 1 < argc) {
            workers_value = static_cast<size_t>(std::stoul(argv[++i]));
            workers_override = true;
        }
        else if (arg == "--max-stages" && i + 1 < argc) max_stages = static_cast<size_t>(std::stoul(argv[++i]));
        else if (arg == "--fallback-timeout" && i + 1 < argc) {
            fallback_timeout_value = std::stod(argv[++i]);
            fallback_timeout_override = true;
        }
        else if (arg == "--skip-baselines") {
            strategy_selection = {false, false, true};
        }
        else if ((arg == "--strategy" || arg == "--strategies") && i + 1 < argc) {
            strategy_selection = parseStrategies(argv[++i]);
        }
        else if (arg == "--h-mode" && i + 1 < argc) {
            h_mode_value = parseHSearchMode(argv[++i]);
            h_mode_override = true;
        }
    }

    LoadedYamlConfig loaded_config = loadConfigFromYaml(config_path);
    UpdownAwareIkConfig experiment_config = loaded_config.ik;
    ComparisonConfig comparison_config = loaded_config.comparison;
    if (timeout_override) experiment_config.timeout = timeout_value;
    if (fallback_timeout_override) experiment_config.fallback_timeout = fallback_timeout_value;
    if (workers_override) experiment_config.workers = workers_value;
    if (h_mode_override) experiment_config.h_search_mode = h_mode_value;
    if (seed_attempts_override) comparison_config.unlimited_seed_attempts = seed_attempts_value;
    const LeverParams lever_params = leverParamsFromConfig(experiment_config);

    const double timeout = experiment_config.timeout;
    const double fallback_timeout = experiment_config.fallback_timeout;
    const double tool0_offset = experiment_config.tool0_offset;

    std::filesystem::create_directories(output.parent_path());
    std::ofstream ofs(output);
    if (!ofs.good()) {
        std::cerr << "Cannot write: " << output << "\n";
        rclcpp::shutdown();
        return 1;
    }

    IkSolverOptions options;
    options.base_frame = experiment_config.base_frame;
    options.tip_link = experiment_config.left_tip;
    options.tip_link2 = experiment_config.right_tip;
    options.reject_collisions = false;
    options.enforce_arm_base_collisions = experiment_config.enforce_arm_base_collisions;

    std::unique_ptr<IkSolver> unlimited_ik;
    std::unique_ptr<IkSolver> lookup_arm_ik;
    std::unique_ptr<IkSolver> lookup_fallback_ik;
    std::unique_ptr<ParallelUpdownAwareIkSolver> lookup_solver;
    std::unique_ptr<ParallelUpdownAwareIkSolver> experiment_solver;

    if (strategy_selection.unlimited) {
        unlimited_ik = std::make_unique<IkSolver>(experiment_config.free_group, experiment_config.solver_plugin, timeout, false, options);
    }
    if (strategy_selection.lookup) {
        lookup_arm_ik = std::make_unique<IkSolver>(experiment_config.fixed_group, experiment_config.solver_plugin, timeout, false, options);
        lookup_fallback_ik = std::make_unique<IkSolver>(experiment_config.free_group, experiment_config.solver_plugin, fallback_timeout, false, options);
    }

    UpdownAwareIkConfig lookup_config = experiment_config;
    lookup_config.timeout = timeout;
    lookup_config.workers = comparison_config.lookup_workers;
    lookup_config.seed_count = comparison_config.lookup_seed_count;
    lookup_config.h_candidate_count = comparison_config.lookup_h_candidate_count;
    lookup_config.tool0_offset = tool0_offset;
    lookup_config.fallback_enabled = true;
    lookup_config.fallback_timeout = fallback_timeout;
    lookup_config.fallback_seed_count = comparison_config.lookup_fallback_seed_count;

    if (strategy_selection.lookup) {
        lookup_solver = std::make_unique<ParallelUpdownAwareIkSolver>(lookup_config);
    }
    if (strategy_selection.experiment) {
        experiment_solver = std::make_unique<ParallelUpdownAwareIkSolver>(experiment_config);
    }

    nlohmann::json header;
    header["type"] = "header";
    header["benchmark"] = "updown_solver_comparison";
    header["config"] = config_path.string();
    header["output"] = output.string();
    header["rounds"] = rounds;
    header["timeout"] = timeout;
    header["unlimited_seed_attempts"] = comparison_config.unlimited_seed_attempts;
    header["unlimited_seed_noise"] = comparison_config.unlimited_seed_noise;
    header["lookup_workers"] = lookup_config.workers;
    header["lookup_h_candidate_count"] = lookup_config.h_candidate_count;
    header["lookup_seed_count"] = lookup_config.seed_count;
    header["lookup_fallback_seed_count"] = lookup_config.fallback_seed_count;
    header["experiment_workers"] = experiment_config.workers;
    header["experiment_h_candidate_count"] = experiment_config.h_candidate_count;
    header["experiment_seed_count"] = experiment_config.seed_count;
    header["experiment_try_target_orders"] = experiment_config.try_target_orders;
    header["experiment_use_reversed_target_order"] = experiment_config.use_reversed_target_order;
    header["experiment_continuous_seed_multiplier"] = experiment_config.continuous_seed_multiplier;
    header["experiment_fallback_rounds"] = experiment_config.fallback_rounds;
    header["experiment_fallback_seed_count"] = experiment_config.fallback_seed_count;
    header["experiment_fallback_random_family_count"] = experiment_config.fallback_random_family_count;
    header["experiment_fallback_random_per_family"] = experiment_config.fallback_random_per_family;
    header["experiment_fallback_seed_noise"] = experiment_config.fallback_seed_noise;
    header["experiment_fallback_updown_noise"] = experiment_config.fallback_updown_noise;
    header["experiment_joint2_torque_weight"] = experiment_config.cost_joint2_torque;
    header["experiment_joint3_torque_weight"] = experiment_config.cost_joint3_torque;
    header["fixed_group"] = experiment_config.fixed_group;
    header["free_group"] = experiment_config.free_group;
    header["solver_plugin"] = experiment_config.solver_plugin;
    header["schema_version"] = 2;
    header["record_candidates"] = strategy_selection.experiment;
    header["candidate_record_mode"] = strategy_selection.experiment ? "all" : "none";
    header["enabled_strategies"] = enabledStrategiesJson(strategy_selection);
    header["experiment_h_search_mode"] = hSearchModeName(experiment_config.h_search_mode);
    header["cost_weights"] = costWeightsJson(experiment_config);
    header["cost_thresholds"] = costThresholdsJson(experiment_config);
    header["check_collision"] = experiment_config.check_collision;
    header["enforce_arm_base_collisions"] = experiment_config.enforce_arm_base_collisions;
    header["max_stages"] = max_stages;
    header["fallback_timeout"] = fallback_timeout;
    header["place_safe_z"] = comparison_config.place_safe_z;
    header["approach_offset"] = comparison_config.approach_offset;
    ofs << header.dump() << "\n";

    const auto points = pickPoints();
    const size_t n_rounds = std::min(rounds, points.size());
    std::map<std::string, double> total_updown;
    std::map<std::string, double> total_wall;
    std::map<std::string, size_t> success_count;
    std::map<std::string, size_t> stage_count;

    std::vector<double> unlimited_seed = unlimited_ik ? std::vector<double>(unlimited_ik->variableNames().size(), 0.0) : std::vector<double>();
    std::vector<double> lookup_arm_seed = lookup_arm_ik ? std::vector<double>(lookup_arm_ik->variableNames().size(), 0.0) : std::vector<double>();
    std::vector<double> lookup_full_seed = lookup_fallback_ik ? std::vector<double>(lookup_fallback_ik->variableNames().size(), 0.0) : std::vector<double>();
    std::vector<double> exp_arm_seed = experiment_solver ? std::vector<double>(experiment_solver->fixedVariableNames().size(), 0.0) : std::vector<double>();
    std::vector<double> exp_full_seed = experiment_solver ? std::vector<double>(experiment_solver->freeVariableNames().size(), 0.0) : std::vector<double>();
    double unlimited_h = 0.0, lookup_h = 0.0, exp_h = 0.0;

    size_t emitted_stages = 0;
    for (size_t round = 0; round < n_rounds; ++round) {
        for (const auto& stage : makeStages(round, points[round], comparison_config.approach_offset, comparison_config.place_safe_z)) {
            if (max_stages > 0 && emitted_stages >= max_stages) {
                break;
            }
            ++emitted_stages;
            std::vector<nlohmann::json> records;
            if (strategy_selection.unlimited) {
                records.push_back(runUnlimitedStage(*unlimited_ik, stage, unlimited_h, unlimited_seed,
                                                    timeout, comparison_config.unlimited_seed_attempts,
                                                    comparison_config.unlimited_seed_noise,
                                                    experiment_config.h_step,
                                                    experiment_config.h_lower,
                                                    experiment_config.h_upper,
                                                    tool0_offset,
                                                    experiment_config.position_tolerance,
                                                    experiment_config.orientation_tolerance,
                                                    lever_params));
            }
            if (strategy_selection.lookup) {
                records.push_back(runLookupStage(*lookup_arm_ik, *lookup_fallback_ik, *lookup_solver, stage, lookup_h,
                                                 lookup_arm_seed, lookup_full_seed,
                                                 lookup_config.seed_count,
                                                 lookup_config.fallback_seed_count,
                                                 lever_params, lookup_config));
            }
            if (strategy_selection.experiment) {
                records.push_back(runNewSolverStage(*experiment_solver, stage, exp_h, exp_arm_seed, exp_full_seed,
                                                    lever_params, experiment_config));
            }

            for (auto& record : records) {
                const std::string strategy = record["strategy"];
                record["type"] = "stage";
                record["round"] = round;
                record["stage"] = stage.name;
                record["target_pose"] = poseJson(stage.left);
                record["target_pose2"] = poseJson(stage.right);
                if (record.value("success", false)) {
                    success_count[strategy]++;
                    if (strategy == "unlimited_bioik_until_collision_free") unlimited_h = record.value("selected_h", unlimited_h);
                    if (strategy == "lookup_like_fixed_h_with_fallback") lookup_h = record.value("selected_h", lookup_h);
                    if (strategy == "parallel_updown_aware_solver") exp_h = record.value("selected_h", exp_h);
                }
                stage_count[strategy]++;
                total_updown[strategy] += record.value("updown_delta", 0.0);
                total_wall[strategy] += record.value("wall_ms", record.value("total_solve_ms", 0.0));
                ofs << record.dump() << "\n";
            }
        }
        if (max_stages > 0 && emitted_stages >= max_stages) {
            break;
        }
    }

    for (const auto& [strategy, count] : stage_count) {
        nlohmann::json summary;
        summary["type"] = "summary";
        summary["strategy"] = strategy;
        summary["stage_count"] = count;
        summary["success_count"] = success_count[strategy];
        summary["total_updown_motion"] = total_updown[strategy];
        summary["total_wall_ms"] = total_wall[strategy];
        ofs << summary.dump() << "\n";
    }

    std::cout << "Results saved to: " << output << "\n";
    rclcpp::shutdown();
    return 0;
}
