#include "alfa_robot_benchmarks/srv/plan_joint_target.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace {

using PlanJointTarget = alfa_robot_benchmarks::srv::PlanJointTarget;

std::vector<moveit_msgs::msg::CollisionObject> makeTwoColumnFourRowBoxStack(
    const std::string& frame_id,
    double front_face_x,
    double depth_x,
    double y_spacing,
    double z_spacing,
    double base_z)
{
    const std::vector<std::vector<int>> rows_top_to_bottom = {
        {7, 6},
        {11, 10},
        {15, 14},
        {19, 18},
    };
    const std::vector<double> ys = {0.5 * y_spacing, -0.5 * y_spacing};
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (size_t row = 0; row < rows_top_to_bottom.size(); ++row) {
        const double z = base_z + z_spacing * static_cast<double>(rows_top_to_bottom.size() - 1 - row);
        for (size_t col = 0; col < rows_top_to_bottom[row].size(); ++col) {
            moveit_msgs::msg::CollisionObject object;
            object.header.frame_id = frame_id;
            object.id = "acceptance_box_" + std::to_string(rows_top_to_bottom[row][col]);
            object.operation = moveit_msgs::msg::CollisionObject::ADD;
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {depth_x, 0.4, 0.4};
            geometry_msgs::msg::Pose pose;
            pose.position.x = front_face_x + 0.5 * depth_x;
            pose.position.y = ys[col];
            pose.position.z = z;
            pose.orientation.w = 1.0;
            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(pose);
            objects.push_back(object);
        }
    }
    return objects;
}


moveit_msgs::msg::CollisionObject makeCenterSeparationPlate(
    const std::string& frame_id,
    double x_min,
    double x_max,
    double y_thickness,
    double z_min,
    double z_max)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = "acceptance_center_separation_plate";
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {std::max(0.001, x_max - x_min), std::max(0.001, y_thickness), std::max(0.001, z_max - z_min)};
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5 * (x_min + x_max);
    pose.position.y = 0.0;
    pose.position.z = 0.5 * (z_min + z_max);
    pose.orientation.w = 1.0;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    return object;
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
        planning_time_ = declare_parameter<double>("planning_time", 8.0);
        planning_attempts_ = declare_parameter<int>("planning_attempts", 20);
        velocity_scale_ = declare_parameter<double>("velocity_scale", 0.25);
        acceleration_scale_ = declare_parameter<double>("acceleration_scale", 0.2);
        joint_goal_tolerance_ = declare_parameter<double>("joint_goal_tolerance", 0.01);
        validate_goal_state_collision_ = declare_parameter<bool>("validate_goal_state_collision", true);
        fixed_updown_ = declare_parameter<double>("fixed_updown", 0.18);
        updown_joint_ = declare_parameter<std::string>("updown_joint", "updown");
        updown_path_tolerance_ = declare_parameter<double>("updown_path_tolerance", 0.001);
        final_target_tolerance_deg_ = declare_parameter<double>("final_target_tolerance_deg", 1.0);
        enable_center_separation_plate_ = declare_parameter<bool>("enable_center_separation_plate", true);
        center_plate_x_min_ = declare_parameter<double>("center_plate_x_min", 0.4);
        center_plate_x_max_ = declare_parameter<double>("center_plate_x_max", 0.76);
        center_plate_y_thickness_ = declare_parameter<double>("center_plate_y_thickness", 0.001);
        center_plate_z_min_ = declare_parameter<double>("center_plate_z_min", 0.0);
        center_plate_z_max_ = declare_parameter<double>("center_plate_z_max", 1.8);

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setNumPlanningAttempts(planning_attempts_);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setGoalJointTolerance(joint_goal_tolerance_);
        move_group_->setStartStateToCurrentState();
        robot_model_ = move_group_->getRobotModel();
        if (!robot_model_) {
            throw std::runtime_error("MoveGroupInterface did not return a robot model");
        }
        if (!robot_model_->hasJointModel(updown_joint_)) {
            throw std::runtime_error("Robot model does not contain joint '" + updown_joint_ + "'");
        }
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description");
        if (planning_scene_monitor_->getPlanningScene()) {
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->startWorldGeometryMonitor();
            planning_scene_monitor_->startStateMonitor("/joint_states");
            planning_scene_monitor_->requestPlanningSceneState();
        } else {
            RCLCPP_WARN(get_logger(), "PlanningSceneMonitor init failed; goal collision validation unavailable");
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
        auto objects = makeTwoColumnFourRowBoxStack(frame_id_, 0.76, 0.3, 0.4, 0.4, 0.2);
        if (enable_center_separation_plate_) {
            objects.push_back(makeCenterSeparationPlate(
                frame_id_, center_plate_x_min_, center_plate_x_max_, center_plate_y_thickness_,
                center_plate_z_min_, center_plate_z_max_));
        }
        planning_scene_interface_.applyCollisionObjects(objects);
        RCLCPP_INFO(get_logger(),
                    "Applied %zu static acceptance collision objects: boxes=8 center_plate=%s x=[%.2f, %.2f] y_thickness=%.4f z=[%.2f, %.2f]",
                    objects.size(), enable_center_separation_plate_ ? "on" : "off",
                    center_plate_x_min_, center_plate_x_max_, center_plate_y_thickness_, center_plate_z_min_, center_plate_z_max_);
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
        if (!isGoalStateValid(goal_state)) {
            response->success = false;
            response->failure_reason = "joint target is out of bounds or colliding";
            response->diagnostics_json = "{}";
            RCLCPP_WARN(get_logger(), "Plan rejected for task=%s: joint target invalid/colliding", request->task_id.c_str());
            return;
        }
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





    bool isGoalStateValid(const moveit::core::RobotState& state) const
    {
        const auto* group = robot_model_->getJointModelGroup(group_name_);
        if (group != nullptr && !state.satisfiesBounds(group)) {
            return false;
        }
        if (!validate_goal_state_collision_) {
            return true;
        }
        if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
            return true;
        }
        planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
        return !scene->isStateColliding(state);
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
    double planning_time_ = 8.0;
    int planning_attempts_ = 20;
    double velocity_scale_ = 0.25;
    double acceleration_scale_ = 0.2;
    double joint_goal_tolerance_ = 0.01;
    bool enable_center_separation_plate_ = true;
    double center_plate_x_min_ = 0.4;
    double center_plate_x_max_ = 0.76;
    double center_plate_y_thickness_ = 0.001;
    double center_plate_z_min_ = 0.0;
    double center_plate_z_max_ = 1.8;
    double fixed_updown_ = 0.18;
    std::string updown_joint_ = "updown";
    double updown_path_tolerance_ = 0.001;
    double final_target_tolerance_deg_ = 1.0;
    bool validate_goal_state_collision_ = true;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
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
