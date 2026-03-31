#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class TrajectoryExecutorNode : public rclcpp::Node
{
public:
    TrajectoryExecutorNode() : Node("trajectory_executor_node")
    {
        // 订阅关节状态
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TrajectoryExecutorNode::jointStateCallback, this, std::placeholders::_1));

        // 订阅轨迹命令
        trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory", 10, std::bind(&TrajectoryExecutorNode::trajectoryCallback, this, std::placeholders::_1));

        // 发布执行状态
        execution_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/trajectory_execution_status", 10);

        // 创建 FollowJointTrajectory 动作客户端
        using namespace std::placeholders;
        trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/left_arm_controller/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "Trajectory Executor Node initialized");
    }

private:
    using FollowJointTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_status_publisher_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;
    sensor_msgs::msg::JointState current_joint_state_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_ = *msg;
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());

        // 检查轨迹是否有效
        if (msg->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
            return;
        }

        // 等待动作服务器可用
        if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // 创建动作目标
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory = *msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

        // 发送动作目标
        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&TrajectoryExecutorNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&TrajectoryExecutorNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&TrajectoryExecutorNode::resultCallback, this, std::placeholders::_1);

        trajectory_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Trajectory sent to action server");
    }

    void goalResponseCallback(const FollowJointTrajectoryGoalHandle::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(const FollowJointTrajectoryGoalHandle::SharedPtr&, 
                         const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {
        // 这里可以处理轨迹执行的反馈信息
        double actual_time = feedback->actual.time_from_start.sec + 
                            feedback->actual.time_from_start.nanosec / 1e9;
        double desired_time = feedback->desired.time_from_start.sec + 
                             feedback->desired.time_from_start.nanosec / 1e9;
        if (desired_time > 0) {
            RCLCPP_INFO(this->get_logger(), "Trajectory execution progress: %.2f%%", 
                       (actual_time / desired_time) * 100.0);
        }
    }

    void resultCallback(const FollowJointTrajectoryGoalHandle::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Trajectory execution aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }

        // 发布执行完成状态
        auto status_msg = std::make_shared<std_msgs::msg::Bool>();
        status_msg->data = false;
        execution_status_publisher_->publish(*status_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
