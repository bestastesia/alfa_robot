/**
 * dual_arm_planner_node.cpp
 *
 * 方案 B：【原生约束】—— setPoseTarget 多末端
 *
 * 核心思路：
 *   使用 setPoseTarget 分别设置两个末端目标，让 MoveIt 内部处理多目标约束。
 *   关键修复：自己订阅 joint_states 构造 RobotState，绕开 CurrentStateMonitor 时间戳问题。
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>

static const std::string PLANNING_GROUP = "dual_arm_with_base";
static const std::string LEFT_TIP       = "leftjoint6_link";
static const std::string RIGHT_TIP      = "rightjoint6_link";
static const std::string BASE_FRAME     = "world";

class DualArmPlannerNode : public rclcpp::Node
{
public:
  explicit DualArmPlannerNode(const rclcpp::NodeOptions & options)
  : Node("dual_arm_planner", options)
  {}

  void init()
  {
    // 自己订阅 joint_states（绕开 CurrentStateMonitor 时间戳问题）
    auto qos = rclcpp::QoS(1).reliable();
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(js_mutex_);
        latest_joint_state_ = msg;
      });

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    // 获取机器人模型
    robot_model_ = move_group_->getRobotModel();
    joint_group_ = robot_model_->getJointModelGroup(PLANNING_GROUP);

    plan_exec_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        resp->success = demo_move();
        resp->message = resp->success ? "success" : "failed";
      });

    RCLCPP_INFO(get_logger(), "DualArmPlannerNode ready (方案B: 原生约束)");
    RCLCPP_INFO(get_logger(), "  Group: %s, Left: %s, Right: %s",
      PLANNING_GROUP.c_str(), LEFT_TIP.c_str(), RIGHT_TIP.c_str());
  }

  /**
   * 获取当前 RobotState（自己构造，不依赖 CurrentStateMonitor）
   */
  moveit::core::RobotStatePtr get_current_robot_state()
  {
    // 等待 joint_states
    sensor_msgs::msg::JointState::SharedPtr js;
    for (int retry = 0; retry < 10 && !js; ++retry) {
      {
        std::lock_guard<std::mutex> lock(js_mutex_);
        js = latest_joint_state_;
      }
      if (!js) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }

    if (!js) {
      RCLCPP_ERROR(get_logger(), "未收到 joint_states");
      return nullptr;
    }

    // 构造 RobotState
    auto state = std::make_shared<moveit::core::RobotState>(robot_model_);
    state->setToDefaultValues();

    for (size_t i = 0; i < js->name.size(); ++i) {
      if (robot_model_->hasJointModel(js->name[i])) {
        state->setJointPositions(js->name[i], &js->position[i]);
      }
    }
    state->update();

    return state;
  }

  /**
   * 从 RobotState 获取 link 的位姿
   */
  bool get_pose_from_state(const std::string& link_name, geometry_msgs::msg::Pose& pose)
  {
    auto state = get_current_robot_state();
    if (!state) return false;

    auto tf = state->getGlobalLinkTransform(link_name);
    pose.position.x = tf.translation().x();
    pose.position.y = tf.translation().y();
    pose.position.z = tf.translation().z();
    Eigen::Quaterniond q(tf.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    return true;
  }

  /**
   * 核心接口：双末端规划
   */
  bool plan_and_execute(const geometry_msgs::msg::Pose& left_pose,
                        const geometry_msgs::msg::Pose& right_pose,
                        bool execute = true)
  {
    // --- Step 1: 用自己构造的 RobotState 设置起始状态 ---
    auto current_state = get_current_robot_state();
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "无法获取当前状态");
      return false;
    }
    move_group_->setStartState(*current_state);

    // --- Step 2: 设置双末端目标 ---
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(left_pose, LEFT_TIP);
    move_group_->setPoseTarget(right_pose, RIGHT_TIP);

    RCLCPP_INFO(get_logger(), "设置双末端目标:");
    RCLCPP_INFO(get_logger(), "  左臂: (%.3f, %.3f, %.3f)",
      left_pose.position.x, left_pose.position.y, left_pose.position.z);
    RCLCPP_INFO(get_logger(), "  右臂: (%.3f, %.3f, %.3f)",
      right_pose.position.x, right_pose.position.y, right_pose.position.z);

    // --- Step 3: 规划 ---
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "规划失败，错误码: %d", plan_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "规划成功，轨迹点数: %zu",
      plan.trajectory_.joint_trajectory.points.size());

    if (!execute) return true;

    // --- Step 4: 执行 ---
    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "执行失败，错误码: %d", exec_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "执行成功");
    return true;
  }

  /**
   * 单臂移动
   */
  bool plan_and_execute_single(bool move_left,
                               const geometry_msgs::msg::Pose& target,
                               bool execute = true)
  {
    geometry_msgs::msg::Pose fixed_pose;
    const std::string& fixed_tip = move_left ? RIGHT_TIP : LEFT_TIP;
    if (!get_pose_from_state(fixed_tip, fixed_pose)) {
      RCLCPP_ERROR(get_logger(), "无法获取 %s 当前位姿", fixed_tip.c_str());
      return false;
    }

    const auto& left_pose  = move_left ? target     : fixed_pose;
    const auto& right_pose = move_left ? fixed_pose : target;

    RCLCPP_INFO(get_logger(), "%s臂运动，%s臂固定",
      move_left ? "左" : "右", move_left ? "右" : "左");

    return plan_and_execute(left_pose, right_pose, execute);
  }

private:
  bool demo_move()
  {
    geometry_msgs::msg::Pose left_current;
    if (!get_pose_from_state(LEFT_TIP, left_current)) {
      RCLCPP_ERROR(get_logger(), "无法获取左臂当前位姿");
      return false;
    }

    auto left_target = left_current;
    left_target.position.z += 0.10;

    RCLCPP_INFO(get_logger(), "Demo: 左臂上移 10cm，右臂固定");
    return plan_and_execute_single(true, left_target, true);
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_exec_srv_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex js_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<DualArmPlannerNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  node->init();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}