#include "alfa_robot_benchmarks/srv/plan_joint_target.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <regex>
#include <set>
#include <string>
#include <vector>

namespace {

using PlanJointTarget = alfa_robot_benchmarks::srv::PlanJointTarget;

std::vector<double> parseVector3(const std::string& text)
{
    std::vector<double> values;
    std::stringstream stream(text);
    double value = 0.0;
    while (stream >> value) values.push_back(value);
    while (values.size() < 3) values.push_back(0.0);
    return values;
}

std::vector<moveit_msgs::msg::CollisionObject> loadCargoFromMujocoScene(
    const std::string& scene_xml,
    const std::string& frame_id,
    size_t max_objects,
    double x_shift,
    double y_shift,
    double z_shift)
{
    std::ifstream input(scene_xml);
    if (!input) return {};
    const std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::regex body_regex(R"(<body\s+name=\"(cargo_[^\"]+)\"\s+pos=\"([^\"]+)\"[\s\S]*?<geom\s+name=\"[^\"]+\"\s+type=\"box\"\s+size=\"([^\"]+)\")");
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (auto it = std::sregex_iterator(xml.begin(), xml.end(), body_regex); it != std::sregex_iterator(); ++it) {
        const auto match = *it;
        const auto pos = parseVector3(match[2].str());
        const auto half = parseVector3(match[3].str());
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = frame_id;
        object.id = match[1].str();
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 2.0 * half[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0 * half[1];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 2.0 * half[2];
        geometry_msgs::msg::Pose pose;
        pose.position.x = pos[0] + x_shift;
        pose.position.y = pos[1] + y_shift;
        pose.position.z = pos[2] + z_shift;
        pose.orientation.w = 1.0;
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        objects.push_back(object);
        if (max_objects > 0 && objects.size() >= max_objects) break;
    }
    return objects;
}

std::vector<moveit_msgs::msg::CollisionObject> makeFallbackBoxStack(
    const std::string& frame_id,
    double front_face_x,
    double depth_x,
    double y_spacing,
    double z_spacing,
    double x_shift,
    double y_shift,
    double z_shift)
{
    const std::vector<std::vector<int>> rows_top_to_bottom = {
        {1, 3, 2, 4},
        {5, 7, 6, 8},
        {9, 11, 10, 12},
        {13, 15, 14, 16},
        {17, 19, 18, 20},
    };
    const std::vector<double> ys = {1.5 * y_spacing, 0.5 * y_spacing, -0.5 * y_spacing, -1.5 * y_spacing};
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
        const double z = 0.2 + z_spacing * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
        for (size_t col = 0; col < rows_top_to_bottom[row].size(); ++col) {
            moveit_msgs::msg::CollisionObject object;
            object.header.frame_id = frame_id;
            object.id = "fallback_box_" + std::to_string(rows_top_to_bottom[row][col]);
            object.operation = moveit_msgs::msg::CollisionObject::ADD;
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {depth_x, 0.4, 0.4};
            geometry_msgs::msg::Pose pose;
            pose.position.x = front_face_x + 0.5 * depth_x + x_shift;
            pose.position.y = ys[col] + y_shift;
            pose.position.z = z + z_shift;
            pose.orientation.w = 1.0;
            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(pose);
            objects.push_back(object);
        }
    }
    return objects;
}

} // namespace

class TemporaryMoveitJointPlanner : public rclcpp::Node {
public:
    TemporaryMoveitJointPlanner(const rclcpp::NodeOptions& options)
        : Node("temporary_moveit_joint_planner", options)
    {}

    void init()
    {
        group_name_ = declare_parameter<std::string>("group_name", "dual_v5_arm_with_base");
        frame_id_ = declare_parameter<std::string>("frame_id", "world");
        service_name_ = declare_parameter<std::string>("service_name", "/alfa_moveit/plan_joint_target");
        planning_time_ = declare_parameter<double>("planning_time", 3.0);
        planning_attempts_ = declare_parameter<int>("planning_attempts", 5);
        velocity_scale_ = declare_parameter<double>("velocity_scale", 0.25);
        acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.2);
        fixed_updown_ = declare_parameter<double>("fixed_updown", 0.18);
        updown_joint_ = declare_parameter<std::string>("updown_joint", "updown");
        updown_path_tolerance_ = declare_parameter<double>("updown_path_tolerance", 0.001);
        final_target_tolerance_deg_ = declare_parameter<double>("final_target_tolerance_deg", 0.5);
        scene_xml_ = declare_parameter<std::string>("mujoco_scene_xml", "/mnt/mydisk/ALFA/alfa_robot/simulation/mujoco/scene.xml");
        max_scene_objects_ = static_cast<size_t>(std::max<int64_t>(0, declare_parameter<int64_t>("max_scene_objects", 20)));
        x_shift_ = declare_parameter<double>("scene_x_shift", -3.84);
        y_shift_ = declare_parameter<double>("scene_y_shift", 0.0);
        z_shift_ = declare_parameter<double>("scene_z_shift", 0.0);

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setNumPlanningAttempts(planning_attempts_);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setStartStateToCurrentState();
        robot_model_ = move_group_->getRobotModel();
        if (!robot_model_) {
            throw std::runtime_error("MoveGroupInterface did not return a robot model");
        }
        if (!robot_model_->hasJointModel(updown_joint_)) {
            throw std::runtime_error("Robot model does not contain joint '" + updown_joint_ + "'");
        }

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                latest_joint_state_ = *msg;
            });

        applyObstacles();
        planner_debug_pub_ = create_publisher<std_msgs::msg::String>("/alfa_debug/planner_received_joint_target", rclcpp::QoS(10).reliable());
        planner_trajectory_debug_pub_ = create_publisher<std_msgs::msg::String>("/alfa_debug/planner_output_trajectory", rclcpp::QoS(10).reliable());
        service_ = create_service<PlanJointTarget>(
            service_name_,
            std::bind(&TemporaryMoveitJointPlanner::handlePlan, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Temporary MoveIt joint planner ready: group=%s service=%s %s=%.3f",
                    group_name_.c_str(), service_name_.c_str(), updown_joint_.c_str(), fixed_updown_);
    }

private:
    void applyObstacles()
    {
        auto objects = loadCargoFromMujocoScene(scene_xml_, frame_id_, max_scene_objects_, x_shift_, y_shift_, z_shift_);
        if (objects.empty()) {
            objects = makeFallbackBoxStack(frame_id_, 0.76, 0.3, 0.4, 0.4, 0.0, 0.0, 0.0);
            RCLCPP_WARN(get_logger(), "No MuJoCo cargo parsed from %s, using fallback box stack", scene_xml_.c_str());
        }
        planning_scene_interface_.applyCollisionObjects(objects);
        RCLCPP_INFO(get_logger(), "Applied %zu temporary collision objects from MuJoCo scene reference", objects.size());
    }

    void handlePlan(const std::shared_ptr<PlanJointTarget::Request> request,
                    std::shared_ptr<PlanJointTarget::Response> response)
    {
        const auto& names = request->joint_target.name;
        const auto& positions = request->joint_target.position;
        if (names.empty() || names.size() != positions.size()) {
            response->success = false;
            response->failure_reason = "invalid joint_target names/positions";
            return;
        }

        const auto start_state = makeStartState(request->joint_target);
        if (!start_state) {
            response->success = false;
            response->failure_reason = "no current joint state for explicit start state";
            return;
        }

        std::map<std::string, double> target;
        moveit::core::RobotState goal_state(*start_state);
        for (size_t i = 0; i < names.size(); ++i) {
            const double value = names[i] == updown_joint_ ? fixed_updown_ : positions[i];
            target[names[i]] = value;
            if (hasVariable(names[i])) {
                goal_state.setVariablePosition(names[i], value);
            }
        }
        target[updown_joint_] = fixed_updown_;
        goal_state.setVariablePosition(updown_joint_, fixed_updown_);
        goal_state.update();
        publishReceivedTargetDebug(*request, target, *start_state);
        move_group_->setStartState(*start_state);
        move_group_->setPathConstraints(makeUpdownPathConstraint());
        move_group_->setJointValueTarget(goal_state);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto result = move_group_->plan(plan);
        move_group_->clearPathConstraints();
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            response->success = false;
            response->failure_reason = "MoveIt planning failed";
            response->diagnostics_json = "{}";
            RCLCPP_WARN(get_logger(), "Plan failed for task=%s", request->task_id.c_str());
            return;
        }
        if (!trajectoryKeepsUpdown(plan.trajectory_.joint_trajectory)) {
            response->success = false;
            response->failure_reason = "planned trajectory does not keep updown fixed";
            response->diagnostics_json = "{}";
            RCLCPP_WARN(get_logger(), "Plan rejected for task=%s: updown is not fixed", request->task_id.c_str());
            return;
        }
        const double final_error = trajectoryFinalTargetErrorDeg(plan.trajectory_.joint_trajectory, target);
        if (final_error > final_target_tolerance_deg_) {
            response->success = false;
            response->failure_reason = "planned trajectory final point does not match joint target";
            response->diagnostics_json = "{}";
            RCLCPP_WARN(get_logger(), "Plan rejected for task=%s: final target error %.3fdeg > %.3fdeg",
                        request->task_id.c_str(), final_error, final_target_tolerance_deg_);
            return;
        }
        response->success = true;
        response->failure_reason.clear();
        response->trajectory = plan.trajectory_.joint_trajectory;
        publishPlannerTrajectoryDebug(*request, response->trajectory);
        response->diagnostics_json = "{\"planner\":\"temporary_moveit_joint_planner_explicit_start\"}";
        RCLCPP_INFO(get_logger(), "Plan success task=%s points=%zu %s=%.3f first_last_delta=%.3fdeg",
                    request->task_id.c_str(), response->trajectory.points.size(),
                    updown_joint_.c_str(), fixed_updown_, firstLastMaxDeltaDeg(response->trajectory));
    }



    void publishReceivedTargetDebug(const PlanJointTarget::Request& request,
                                    const std::map<std::string, double>& target,
                                    const moveit::core::RobotState& start_state)
    {
        nlohmann::json data;
        data["source"] = "temporary_moveit_joint_planner";
        data["type"] = "planner_received_joint_target";
        data["task_id"] = request.task_id;
        data["request_names"] = request.joint_target.name;
        data["request_positions_rad"] = request.joint_target.position;
        std::vector<double> request_deg;
        for (double value : request.joint_target.position) request_deg.push_back(value * 180.0 / M_PI);
        data["request_positions_deg"] = request_deg;
        for (const auto& [name, value] : target) {
            data["target_map_rad"][name] = value;
            data["target_map_deg"][name] = value * 180.0 / M_PI;
        }
        const auto& variable_names = robot_model_->getVariableNames();
        for (const auto& name : variable_names) {
            data["start_state_rad"][name] = start_state.getVariablePosition(name);
            data["start_state_deg"][name] = start_state.getVariablePosition(name) * 180.0 / M_PI;
        }
        std_msgs::msg::String msg;
        msg.data = data.dump();
        planner_debug_pub_->publish(msg);
    }

    void publishPlannerTrajectoryDebug(const PlanJointTarget::Request& request,
                                       const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        nlohmann::json data;
        data["source"] = "temporary_moveit_joint_planner";
        data["type"] = "planner_output_trajectory";
        data["task_id"] = request.task_id;
        data["joint_names"] = trajectory.joint_names;
        data["point_count"] = trajectory.points.size();
        if (!trajectory.points.empty()) {
            data["first_positions_rad"] = trajectory.points.front().positions;
            data["last_positions_rad"] = trajectory.points.back().positions;
            std::vector<double> first_deg;
            std::vector<double> last_deg;
            for (double value : trajectory.points.front().positions) first_deg.push_back(value * 180.0 / M_PI);
            for (double value : trajectory.points.back().positions) last_deg.push_back(value * 180.0 / M_PI);
            data["first_positions_deg"] = first_deg;
            data["last_positions_deg"] = last_deg;
        }
        std_msgs::msg::String msg;
        msg.data = data.dump();
        planner_trajectory_debug_pub_->publish(msg);
    }



    bool hasVariable(const std::string& name) const
    {
        const auto& variable_names = robot_model_->getVariableNames();
        return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
    }

    std::optional<moveit::core::RobotState> makeStartState(const sensor_msgs::msg::JointState& fallback_target) const
    {
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();

        sensor_msgs::msg::JointState latest;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            latest = latest_joint_state_;
        }
        const std::set<std::string> variable_names(
            robot_model_->getVariableNames().begin(), robot_model_->getVariableNames().end());
        const auto& source = latest.name.empty() ? fallback_target : latest;
        for (size_t i = 0; i < source.name.size() && i < source.position.size(); ++i) {
            if (variable_names.count(source.name[i]) > 0) {
                state.setVariablePosition(source.name[i], source.position[i]);
            }
        }
        state.setVariablePosition(updown_joint_, fixed_updown_);
        state.update();
        return state;
    }

    moveit_msgs::msg::Constraints makeUpdownPathConstraint() const
    {
        moveit_msgs::msg::Constraints constraints;
        constraints.name = "keep_updown_fixed";
        moveit_msgs::msg::JointConstraint joint_constraint;
        joint_constraint.joint_name = updown_joint_;
        joint_constraint.position = fixed_updown_;
        joint_constraint.tolerance_above = updown_path_tolerance_;
        joint_constraint.tolerance_below = updown_path_tolerance_;
        joint_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint_constraint);
        return constraints;
    }

    bool trajectoryKeepsUpdown(const trajectory_msgs::msg::JointTrajectory& trajectory) const
    {
        const auto it = std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), updown_joint_);
        if (it == trajectory.joint_names.end()) {
            return true;
        }
        const size_t index = static_cast<size_t>(std::distance(trajectory.joint_names.begin(), it));
        for (const auto& point : trajectory.points) {
            if (point.positions.size() <= index) continue;
            if (std::abs(point.positions[index] - fixed_updown_) > updown_path_tolerance_ + 1e-9) {
                return false;
            }
        }
        return true;
    }



    double trajectoryFinalTargetErrorDeg(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                         const std::map<std::string, double>& target) const
    {
        if (trajectory.points.empty()) return std::numeric_limits<double>::infinity();
        const auto& last = trajectory.points.back();
        double max_error = 0.0;
        for (size_t i = 0; i < trajectory.joint_names.size() && i < last.positions.size(); ++i) {
            const auto target_it = target.find(trajectory.joint_names[i]);
            if (target_it == target.end()) continue;
            max_error = std::max(max_error, std::abs(last.positions[i] - target_it->second) * 180.0 / M_PI);
        }
        return max_error;
    }

    double firstLastMaxDeltaDeg(const trajectory_msgs::msg::JointTrajectory& trajectory) const
    {
        if (trajectory.points.size() < 2) return 0.0;
        const auto& first = trajectory.points.front().positions;
        const auto& last = trajectory.points.back().positions;
        const size_t count = std::min(first.size(), last.size());
        double max_delta = 0.0;
        for (size_t i = 0; i < count; ++i) {
            max_delta = std::max(max_delta, std::abs(last[i] - first[i]) * 180.0 / M_PI);
        }
        return max_delta;
    }

    std::string group_name_;
    std::string frame_id_;
    std::string service_name_;
    std::string scene_xml_;
    double planning_time_ = 3.0;
    int planning_attempts_ = 5;
    double velocity_scale_ = 0.25;
    double acceleration_scale_ = 0.2;
    double fixed_updown_ = 0.18;
    std::string updown_joint_ = "updown";
    double updown_path_tolerance_ = 0.001;
    double final_target_tolerance_deg_ = 0.5;
    size_t max_scene_objects_ = 20;
    double x_shift_ = -3.84;
    double y_shift_ = 0.0;
    double z_shift_ = 0.0;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    mutable std::mutex joint_state_mutex_;
    sensor_msgs::msg::JointState latest_joint_state_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_trajectory_debug_pub_;
    rclcpp::Service<PlanJointTarget>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemporaryMoveitJointPlanner>(rclcpp::NodeOptions());
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
