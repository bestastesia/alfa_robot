#include "alfa_robot_benchmarks/msg/task_command.hpp"
#include "alfa_robot_benchmarks/msg/task_status.hpp"
#include "alfa_robot_benchmarks/srv/plan_joint_target.hpp"
#include "alfa_robot_benchmarks/srv/solve_dual_ik.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

using TaskCommand = alfa_robot_benchmarks::msg::TaskCommand;
using TaskStatus = alfa_robot_benchmarks::msg::TaskStatus;
using SolveDualIk = alfa_robot_benchmarks::srv::SolveDualIk;
using PlanJointTarget = alfa_robot_benchmarks::srv::PlanJointTarget;

constexpr double kWorldToBaseZ = 0.202094;

sensor_msgs::msg::JointState homeJointState(double updown)
{
    sensor_msgs::msg::JointState state;
    state.name = {
        "turn", "updown",
        "left_v5_joint1", "left_v5_joint2", "left_v5_joint3", "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
        "right_v5_joint1", "right_v5_joint2", "right_v5_joint3", "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
    };
    state.position = {
        0.0, updown,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    };
    return state;
}


sensor_msgs::msg::JointState fixedDemoJointTarget(double updown)
{
    sensor_msgs::msg::JointState state;
    state.name = {
        "updown",
        "left_v5_joint1", "left_v5_joint2", "left_v5_joint3", "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
        "right_v5_joint1", "right_v5_joint2", "right_v5_joint3", "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
    };
    const double d = M_PI / 180.0;
    state.position = {
        updown,
        -28.0 * d, 51.0 * d, -38.0 * d, 30.0 * d, -81.0 * d, 83.0 * d,
        28.0 * d, 49.0 * d, -35.0 * d, -28.0 * d, -79.0 * d, -85.0 * d,
    };
    return state;
}

void forceUpdown(sensor_msgs::msg::JointState& state, double updown)
{
    for (size_t i = 0; i < state.name.size() && i < state.position.size(); ++i) {
        if (state.name[i] == "updown") {
            state.position[i] = updown;
            return;
        }
    }
    state.name.push_back("updown");
    state.position.push_back(updown);
}

nlohmann::json jointStateToJson(const sensor_msgs::msg::JointState& state)
{
    nlohmann::json data;
    data["names"] = state.name;
    data["positions_rad"] = state.position;
    std::vector<double> degrees;
    degrees.reserve(state.position.size());
    for (double value : state.position) degrees.push_back(value * 180.0 / M_PI);
    data["positions_deg"] = degrees;
    return data;
}

TaskCommand toBaseFrame(TaskCommand command)
{
    if (command.header.frame_id == "world" || command.header.frame_id.empty()) {
        command.left_target.position.z -= kWorldToBaseZ;
        command.right_target.position.z -= kWorldToBaseZ;
        command.header.frame_id = "base_link";
    }
    return command;
}

trajectory_msgs::msg::JointTrajectory makeMockTrajectory(const sensor_msgs::msg::JointState& target)
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = target.name;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target.position;
    point.time_from_start.sec = 2;
    trajectory.points.push_back(point);
    return trajectory;
}

} // namespace

class FixedPlatformTaskOrchestrator : public rclcpp::Node {
public:
    FixedPlatformTaskOrchestrator()
        : Node("fixed_platform_task_orchestrator")
    {
        task_topic_ = declare_parameter<std::string>("task_topic", "/alfa_task/command");
        status_topic_ = declare_parameter<std::string>("status_topic", "/alfa_task/status");
        ik_service_name_ = declare_parameter<std::string>("ik_service", "/alfa_dual_ik/solve");
        planner_service_name_ = declare_parameter<std::string>("planner_service", "/alfa_moveit/plan_joint_target");
        trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "/plc_joint_trajectory");
        plc_state_topic_ = declare_parameter<std::string>("plc_state_topic", "/plc_bridge_state");
        fixed_updown_ = declare_parameter<double>("fixed_updown", 0.18);
        mock_planner_ = declare_parameter<bool>("mock_planner", true);
        wait_execution_done_ = declare_parameter<bool>("wait_execution_done", true);
        execution_timeout_ms_ = declare_parameter<int>("execution_timeout_ms", 120000);
        service_timeout_ms_ = declare_parameter<int>("service_timeout_ms", 30000);
        ik_max_attempts_ = static_cast<int>(std::max<int64_t>(1, declare_parameter<int64_t>("ik_max_attempts", 5)));
        demo_mode_ = declare_parameter<std::string>("demo_mode", "ik");

        latest_joint_state_ = homeJointState(fixed_updown_);

        status_pub_ = create_publisher<TaskStatus>(status_topic_, rclcpp::QoS(10).reliable());
        trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(trajectory_topic_, rclcpp::QoS(10).reliable());
        ik_debug_pub_ = create_publisher<std_msgs::msg::String>("/alfa_debug/orchestrator_ik_result", rclcpp::QoS(10).reliable());
        trajectory_debug_pub_ = create_publisher<std_msgs::msg::String>("/alfa_debug/orchestrator_plc_trajectory", rclcpp::QoS(10).reliable());
        task_sub_ = create_subscription<TaskCommand>(
            task_topic_, rclcpp::QoS(10).reliable(),
            [this](const TaskCommand::SharedPtr msg) { handleTask(*msg); });
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                latest_joint_state_ = *msg;
                forceUpdown(latest_joint_state_, fixed_updown_);
            });
        plc_state_sub_ = create_subscription<std_msgs::msg::String>(
            plc_state_topic_, rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(plc_state_mutex_);
                latest_plc_state_ = msg->data;
            });

        ik_client_ = create_client<SolveDualIk>(ik_service_name_);
        planner_client_ = create_client<PlanJointTarget>(planner_service_name_);

        RCLCPP_INFO(get_logger(), "Task orchestrator ready: task=%s status=%s IK=%s planner=%s%s trajectory_topic=%s",
                    task_topic_.c_str(), status_topic_.c_str(), ik_service_name_.c_str(),
                    planner_service_name_.c_str(), mock_planner_ ? "(mock)" : "",
                    trajectory_topic_.c_str());
    }

private:
    void handleTask(const TaskCommand& command)
    {
        if (busy_) {
            publishStatus(command, "rejected", "orchestrator is busy");
            return;
        }
        if (last_done_task_id_ == command.task_id) {
            publishStatus(command, "done", "duplicate task already completed");
            return;
        }
        if (current_task_id_ == command.task_id) {
            publishStatus(command, "accepted", "duplicate task already accepted");
            return;
        }

        busy_ = true;
        current_task_id_ = command.task_id;
        RCLCPP_INFO(get_logger(), "Task accepted %s: L%d=(%.3f, %.3f, %.3f) R%d=(%.3f, %.3f, %.3f) frame=%s mode=%s",
                    command.task_id.c_str(),
                    command.left_box_id, command.left_target.position.x, command.left_target.position.y, command.left_target.position.z,
                    command.right_box_id, command.right_target.position.x, command.right_target.position.y, command.right_target.position.z,
                    command.header.frame_id.c_str(), command.grasp_mode.c_str());
        publishStatus(command, "accepted", "task received");
        std::thread([this, command]() { processTask(command); }).detach();
    }

    void processTask(TaskCommand command)
    {
        const auto base_command = toBaseFrame(command);
        auto ik_request = std::make_shared<SolveDualIk::Request>();
        ik_request->header = base_command.header;
        ik_request->left_target = base_command.left_target;
        ik_request->right_target = base_command.right_target;
        ik_request->grasp_mode = base_command.grasp_mode.empty() ? "front" : base_command.grasp_mode;
        ik_request->current_updown = fixed_updown_;
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            ik_request->current_joint_state = latest_joint_state_;
        }
        forceUpdown(ik_request->current_joint_state, fixed_updown_);

        sensor_msgs::msg::JointState selected_joint_target;
        if (demo_mode_ == "fixed_joint_target") {
            selected_joint_target = fixedDemoJointTarget(fixed_updown_);
            selected_joint_target.header = base_command.header;
            RCLCPP_WARN(get_logger(), "Demo mode fixed_joint_target: skipping IK and using hardcoded 12-axis target for task=%s",
                        command.task_id.c_str());
            publishFixedTargetDebug(command, selected_joint_target);
        } else {
            publishStatus(command, "running", "calling IK");
            SolveDualIk::Response::SharedPtr ik_response;
            std::string last_ik_reason;
            for (int attempt = 1; attempt <= ik_max_attempts_; ++attempt) {
                ik_response = callService<SolveDualIk>(ik_client_, ik_request, ik_service_name_);
                if (ik_response && ik_response->success) {
                    if (attempt > 1) {
                        RCLCPP_INFO(get_logger(), "IK succeeded on attempt %d/%d for task %s", attempt, ik_max_attempts_, command.task_id.c_str());
                    }
                    break;
                }
                last_ik_reason = ik_response ? ik_response->failure_reason : "IK service unavailable/timeout";
                RCLCPP_WARN(get_logger(), "IK attempt %d/%d failed for task %s: %s",
                            attempt, ik_max_attempts_, command.task_id.c_str(), last_ik_reason.c_str());
            }
            if (!ik_response || !ik_response->success) {
                failTask(command, "IK failed after " + std::to_string(ik_max_attempts_) + " attempts: " + last_ik_reason);
                return;
            }
            selected_joint_target = ik_response->joint_target;
            publishIkDebug(command, *ik_response);
        }

        publishStatus(command, "running", "calling MoveIt planner");
        trajectory_msgs::msg::JointTrajectory trajectory;
        if (mock_planner_) {
            trajectory = makeMockTrajectory(selected_joint_target);
        } else {
            auto plan_request = std::make_shared<PlanJointTarget::Request>();
            plan_request->header = base_command.header;
            plan_request->task_id = command.task_id;
            plan_request->joint_target = selected_joint_target;
            auto plan_response = callService<PlanJointTarget>(planner_client_, plan_request, planner_service_name_);
            if (!plan_response || !plan_response->success) {
                const std::string reason = plan_response ? plan_response->failure_reason : "planner service unavailable/timeout";
                failTask(command, "MoveIt planning failed: " + reason);
                return;
            }
            trajectory = plan_response->trajectory;
        }

        publishStatus(command, "running", "publishing trajectory to PLC bridge");
        trajectory.header = base_command.header;
        trajectory.header.stamp = now();
        publishTrajectoryDebug(command, trajectory);
        RCLCPP_INFO(get_logger(),
                    "Publishing PLC trajectory task=%s topic=%s points=%zu joints=%zu subscribers=%zu",
                    command.task_id.c_str(), trajectory_topic_.c_str(), trajectory.points.size(),
                    trajectory.joint_names.size(), trajectory_pub_->get_subscription_count());
        {
            std::lock_guard<std::mutex> lock(plc_state_mutex_);
            latest_plc_state_.clear();
        }
        trajectory_pub_->publish(trajectory);
        RCLCPP_INFO(get_logger(), "Published PLC trajectory task=%s", command.task_id.c_str());
        if (wait_execution_done_ && !waitForExecutionDone(command)) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            latest_joint_state_ = selected_joint_target;
            forceUpdown(latest_joint_state_, fixed_updown_);
        }
        last_done_task_id_ = command.task_id;
        current_task_id_.clear();
        busy_ = false;
        publishStatus(command, "done", "task completed");
    }




    void publishFixedTargetDebug(const TaskCommand& command, const sensor_msgs::msg::JointState& joint_target)
    {
        nlohmann::json data;
        data["source"] = "fixed_platform_task_orchestrator";
        data["type"] = "fixed_joint_target_to_planner";
        data["task_id"] = command.task_id;
        data["success"] = true;
        data["failure_reason"] = "";
        data["selected_h"] = fixed_updown_;
        data["score"] = 0.0;
        data["wall_ms"] = 0.0;
        data["joint_target"] = jointStateToJson(joint_target);
        std_msgs::msg::String msg;
        msg.data = data.dump();
        ik_debug_pub_->publish(msg);
    }

    void publishIkDebug(const TaskCommand& command, const SolveDualIk::Response& ik_response)
    {
        nlohmann::json data;
        data["source"] = "fixed_platform_task_orchestrator";
        data["type"] = "ik_result_to_planner";
        data["task_id"] = command.task_id;
        data["success"] = ik_response.success;
        data["failure_reason"] = ik_response.failure_reason;
        data["selected_h"] = ik_response.selected_h;
        data["score"] = ik_response.score;
        data["wall_ms"] = ik_response.wall_ms;
        data["joint_target"] = jointStateToJson(ik_response.joint_target);
        std_msgs::msg::String msg;
        msg.data = data.dump();
        ik_debug_pub_->publish(msg);
    }

    void publishTrajectoryDebug(const TaskCommand& command, const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        nlohmann::json data;
        data["source"] = "fixed_platform_task_orchestrator";
        data["type"] = "trajectory_to_plc";
        data["task_id"] = command.task_id;
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
        trajectory_debug_pub_->publish(msg);
    }

    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callService(
        const typename rclcpp::Client<ServiceT>::SharedPtr& client,
        const typename ServiceT::Request::SharedPtr& request,
        const std::string& service_name)
    {
        if (!client->wait_for_service(std::chrono::milliseconds(service_timeout_ms_))) {
            RCLCPP_ERROR(get_logger(), "Service not available: %s", service_name.c_str());
            return nullptr;
        }
        auto future = client->async_send_request(request);
        const auto status = future.wait_for(std::chrono::milliseconds(service_timeout_ms_));
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Service timeout: %s", service_name.c_str());
            return nullptr;
        }
        return future.get();
    }

    void publishStatus(const TaskCommand& command, const std::string& state, const std::string& message)
    {
        TaskStatus status;
        status.header.stamp = now();
        status.header.frame_id = command.header.frame_id;
        status.task_id = command.task_id;
        status.sequence_index = command.sequence_index;
        status.state = state;
        status.message = message;
        status_pub_->publish(status);
        RCLCPP_INFO(get_logger(), "Task %s: %s - %s", command.task_id.c_str(), state.c_str(), message.c_str());
    }

    void failTask(const TaskCommand& command, const std::string& message)
    {
        current_task_id_.clear();
        busy_ = false;
        publishStatus(command, "failed", message);
    }

    bool waitForExecutionDone(const TaskCommand& command)
    {
        const auto start = std::chrono::steady_clock::now();
        bool saw_executing = false;
        while (rclcpp::ok()) {
            std::string state;
            {
                std::lock_guard<std::mutex> lock(plc_state_mutex_);
                state = latest_plc_state_;
            }
            const bool executing = state.find("executing=true") != std::string::npos ||
                                   state.find("state=executing") != std::string::npos;
            const bool normal_idle = state.find("state=normal") != std::string::npos &&
                                     state.find("executing=false") != std::string::npos;
            const bool command_reported = state.find("last_command_count=") != std::string::npos &&
                                          state.find("last_command_count=0") == std::string::npos;
            if (executing) {
                saw_executing = true;
            }
            if ((saw_executing || command_reported) && normal_idle) {
                return true;
            }
            if (state.find("state=fault") != std::string::npos ||
                state.find("state=emergency_stopped") != std::string::npos ||
                state.find("state=plc_comm_error") != std::string::npos ||
                state.find("state=soft_stopped") != std::string::npos) {
                failTask(command, "PLC execution failed: " + state);
                return false;
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > execution_timeout_ms_) {
                const std::string phase = saw_executing ? "finish" : "start";
                failTask(command, "PLC execution " + phase + " timeout; last state: " + state);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        failTask(command, "ROS shutdown while waiting PLC execution");
        return false;
    }

    std::string task_topic_;
    std::string status_topic_;
    std::string ik_service_name_;
    std::string planner_service_name_;
    std::string trajectory_topic_;
    std::string plc_state_topic_;
    double fixed_updown_ = 0.18;
    bool mock_planner_ = true;
    bool wait_execution_done_ = true;
    int execution_timeout_ms_ = 120000;
    int service_timeout_ms_ = 30000;
    int ik_max_attempts_ = 5;
    std::string demo_mode_ = "ik";

    bool busy_ = false;
    std::string current_task_id_;
    std::string last_done_task_id_;
    std::mutex joint_mutex_;
    std::mutex plc_state_mutex_;
    sensor_msgs::msg::JointState latest_joint_state_;
    std::string latest_plc_state_;

    rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ik_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_debug_pub_;
    rclcpp::Subscription<TaskCommand>::SharedPtr task_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plc_state_sub_;
    rclcpp::Client<SolveDualIk>::SharedPtr ik_client_;
    rclcpp::Client<PlanJointTarget>::SharedPtr planner_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedPlatformTaskOrchestrator>());
    rclcpp::shutdown();
    return 0;
}
