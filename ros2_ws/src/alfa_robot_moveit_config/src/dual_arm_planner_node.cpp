/**
 * dual_arm_planner_node.cpp
 *
 * 方案 A：【编程拖球】—— 100% 模拟 RViz 内部工作流
 *
 * 核心步骤：
 *   1. 自己订阅 joint_states 构造 RobotState（绕开 CurrentStateMonitor 时间戳问题）
 *   2. 获取底层 JointModelGroup
 *   3. setFromIK 多末端重载，传入双末端目标
 *   4. 检查结果
 *   5. setJointValueTarget(*state) 设置关节目标
 *   6. plan() 执行规划
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <random>

static const std::string PLANNING_GROUP = "dual_arm_with_base";
static const std::string LEFT_TIP       = "left_v5_tool0";
static const std::string RIGHT_TIP      = "right_v5_tool0";

class DualArmPlannerNode : public rclcpp::Node
{
public:
  explicit DualArmPlannerNode(const rclcpp::NodeOptions & options)
  : Node("dual_arm_planner", options)
  {}

  void init()
  {
    // 自己订阅 joint_states（绕开 CurrentStateMonitor 时间戳问题）
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
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

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    if (!planning_scene_monitor_->getPlanningScene()) {
      RCLCPP_WARN(get_logger(), "PlanningSceneMonitor 初始化失败，IK 碰撞过滤不可用");
    } else {
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->requestPlanningSceneState();
      RCLCPP_INFO(get_logger(), "PlanningSceneMonitor ready: IK 将拒绝碰撞状态");
    }

    plan_exec_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        resp->success = demo_move();
        resp->message = resp->success ? "success" : "failed";
      });

    RCLCPP_INFO(get_logger(), "DualArmPlannerNode ready (方案A: RViz IK 风格)");
    RCLCPP_INFO(get_logger(), "  Group: %s, Left: %s, Right: %s",
      PLANNING_GROUP.c_str(), LEFT_TIP.c_str(), RIGHT_TIP.c_str());
  }

private:
  /**
   * 获取当前 RobotState（自己构造，绕开 CurrentStateMonitor 时间戳问题）
   */
  moveit::core::RobotStatePtr get_current_robot_state()
  {
    sensor_msgs::msg::JointState::SharedPtr js;
    for (int retry = 0; retry < 20; ++retry) {
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        js = latest_joint_state_;
      }
      if (js) break;
      RCLCPP_DEBUG(get_logger(), "等待 joint_states... (%d/20)", retry + 1);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!js) {
      RCLCPP_ERROR(get_logger(), "未收到 joint_states");
      return nullptr;
    }

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
   * 核心接口：双末端位姿规划 + 执行
   * 100% 模拟 RViz 拖球内部流程
   */
  bool plan_and_execute(const geometry_msgs::msg::Pose & left_pose,
                        const geometry_msgs::msg::Pose & right_pose,
                        bool execute = true)
  {
    // ========== Step 1: 获取当前绝对可靠的起始种子 ==========
    RCLCPP_INFO(get_logger(), "Step 1: 获取当前 RobotState 作为种子...");
    moveit::core::RobotStatePtr current_state = get_current_robot_state();
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "无法获取当前机器人状态");
      return false;
    }

    // 打印当前关节角度
    std::vector<double> current_joints;
    current_state->copyJointGroupPositions(joint_group_, current_joints);
    auto& jnames = joint_group_->getVariableNames();
    RCLCPP_INFO(get_logger(), "当前种子状态 (前5个关节):");
    for (size_t i = 0; i < jnames.size() && i < 5; ++i) {
      RCLCPP_INFO(get_logger(), "  %s = %.4f", jnames[i].c_str(), current_joints[i]);
    }

    // 打印当前末端位姿
    auto left_tf = current_state->getGlobalLinkTransform(LEFT_TIP);
    auto right_tf = current_state->getGlobalLinkTransform(RIGHT_TIP);
    RCLCPP_INFO(get_logger(), "当前左臂末端: (%.3f, %.3f, %.3f)",
      left_tf.translation().x(), left_tf.translation().y(), left_tf.translation().z());
    RCLCPP_INFO(get_logger(), "当前右臂末端: (%.3f, %.3f, %.3f)",
      right_tf.translation().x(), right_tf.translation().y(), right_tf.translation().z());

    // ========== Step 2: 提取底层 Kinematics 求解器实例 ==========
    // joint_group_ 已在 init() 中获取

    // ========== Step 3: 调用底层的多末端 setFromIK ==========
    RCLCPP_INFO(get_logger(), "Step 3: 调用 setFromIK 多末端求解...");
    RCLCPP_INFO(get_logger(), "  左臂目标: (%.3f, %.3f, %.3f)",
      left_pose.position.x, left_pose.position.y, left_pose.position.z);
    RCLCPP_INFO(get_logger(), "  右臂目标: (%.3f, %.3f, %.3f)",
      right_pose.position.x, right_pose.position.y, right_pose.position.z);

    // 构造目标 Pose 数组
    EigenSTL::vector_Isometry3d poses(2);
    poses[0] = Eigen::Translation3d(left_pose.position.x, left_pose.position.y, left_pose.position.z)
               * Eigen::Quaterniond(left_pose.orientation.w, left_pose.orientation.x,
                                   left_pose.orientation.y, left_pose.orientation.z);
    poses[1] = Eigen::Translation3d(right_pose.position.x, right_pose.position.y, right_pose.position.z)
               * Eigen::Quaterniond(right_pose.orientation.w, right_pose.orientation.x,
                                   right_pose.orientation.y, right_pose.orientation.z);

    std::vector<std::string> tips = {LEFT_TIP, RIGHT_TIP};

    auto validity_callback =
      [this](moveit::core::RobotState* robot_state,
             const moveit::core::JointModelGroup* joint_group,
             const double* joint_group_variable_values) {
        robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
        robot_state->update();

        if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
          return robot_state->satisfiesBounds(joint_group);
        }

        planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
        return robot_state->satisfiesBounds(joint_group) &&
               !scene->isStateColliding(*robot_state, joint_group->getName());
      };

    // 关键：调用 setFromIK，timeout = 2.0 秒让 BioIK 充分进化，并拒绝碰撞 IK 解
    double ik_timeout = 2.0;
    bool ik_success = current_state->setFromIK(
      joint_group_, poses, tips, ik_timeout, validity_callback);

    // ========== Step 4: 检查解算结果 ==========
    if (!ik_success) {
      RCLCPP_ERROR(get_logger(), "Step 4: setFromIK 求解失败！目标可能超出物理极限");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Step 4: setFromIK 求解成功！");

    // 打印 IK 解
    std::vector<double> ik_joints;
    current_state->copyJointGroupPositions(joint_group_, ik_joints);
    RCLCPP_INFO(get_logger(), "IK 求解成功，关节角度:");
    for (size_t i = 0; i < jnames.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  %s = %.4f", jnames[i].c_str(), ik_joints[i]);
    }

    // ========== Step 5: 下发稳赚不赔的关节目标 ==========
    RCLCPP_INFO(get_logger(), "Step 5: 设置关节目标...");
    move_group_->setJointValueTarget(*current_state);

    // ========== Step 6: 执行规划 ==========
    RCLCPP_INFO(get_logger(), "Step 6: 执行规划...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "规划失败，错误码: %d", plan_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "规划成功，轨迹点数: %zu",
      plan.trajectory_.joint_trajectory.points.size());

    if (!execute) return true;

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "执行失败，错误码: %d", exec_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "执行成功！");
    return true;
  }

  /**
   * 单臂移动（另一臂保持当前位姿）
   */
  bool plan_and_execute_single(bool move_left,
                               const geometry_msgs::msg::Pose & target,
                               bool execute = true)
  {
    auto current_state = get_current_robot_state();
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "无法获取当前机器人状态");
      return false;
    }

    const std::string& fixed_tip = move_left ? RIGHT_TIP : LEFT_TIP;
    auto fixed_tf = current_state->getGlobalLinkTransform(fixed_tip);

    geometry_msgs::msg::Pose fixed_pose;
    fixed_pose.position.x = fixed_tf.translation().x();
    fixed_pose.position.y = fixed_tf.translation().y();
    fixed_pose.position.z = fixed_tf.translation().z();
    Eigen::Quaterniond q(fixed_tf.rotation());
    fixed_pose.orientation.w = q.w();
    fixed_pose.orientation.x = q.x();
    fixed_pose.orientation.y = q.y();
    fixed_pose.orientation.z = q.z();

    const auto& left_pose  = move_left ? target : fixed_pose;
    const auto& right_pose = move_left ? fixed_pose : target;

    RCLCPP_INFO(get_logger(), "%s臂运动，%s臂固定",
      move_left ? "左" : "右", move_left ? "右" : "左");

    return plan_and_execute(left_pose, right_pose, execute);
  }

  /**
   * Demo：双臂相对移动 + 左臂随机扰动
   * 左臂：X+5cm, Y+5cm, Z+10cm + 随机扰动
   * 右臂：Z+50cm，姿态朝前（无扰动）
   */
  bool demo_move()
  {
    auto current_state = get_current_robot_state();
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "无法获取当前机器人状态");
      return false;
    }

    // 随机数生成器（小扰动）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.02, 0.02);  // ±2cm 扰动
    std::uniform_real_distribution<> angle_dis(-0.1, 0.1);  // ±0.1 弧度扰动

    // 获取当前末端位姿
    auto left_tf = current_state->getGlobalLinkTransform(LEFT_TIP);
    auto right_tf = current_state->getGlobalLinkTransform(RIGHT_TIP);

    // 左臂目标：X+5cm, Y+5cm, Z+10cm + 随机扰动
    geometry_msgs::msg::Pose left_target;
    left_target.position.x = left_tf.translation().x() + 0.05 + dis(gen);
    left_target.position.y = left_tf.translation().y() + 0.05 + dis(gen);
    left_target.position.z = left_tf.translation().z() + 0.10 + dis(gen);
    // 姿态保持当前 + 小扰动
    Eigen::Quaterniond q_left(left_tf.rotation());
    Eigen::AngleAxisd rot_left(angle_dis(gen), Eigen::Vector3d(angle_dis(gen), angle_dis(gen), angle_dis(gen)).normalized());
    q_left = q_left * Eigen::Quaterniond(rot_left);
    left_target.orientation.w = q_left.w();
    left_target.orientation.x = q_left.x();
    left_target.orientation.y = q_left.y();
    left_target.orientation.z = q_left.z();

    // 右臂目标：Z+50cm，姿态朝前（无扰动）
    geometry_msgs::msg::Pose right_target;
    right_target.position.x = right_tf.translation().x();
    right_target.position.y = right_tf.translation().y();
    right_target.position.z = right_tf.translation().z() + 0.5;
    // 姿态：水平向前
    Eigen::Quaterniond q_forward;
    q_forward = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
    right_target.orientation.w = q_forward.w();
    right_target.orientation.x = q_forward.x();
    right_target.orientation.y = q_forward.y();
    right_target.orientation.z = q_forward.z();

    RCLCPP_INFO(get_logger(), "Demo: 左臂 Z+10cm + 扰动, 右臂姿态朝前");
    RCLCPP_INFO(get_logger(), "  左臂目标: (%.3f, %.3f, %.3f)",
      left_target.position.x, left_target.position.y, left_target.position.z);
    RCLCPP_INFO(get_logger(), "  右臂目标: (%.3f, %.3f, %.3f)",
      right_target.position.x, right_target.position.y, right_target.position.z);

    return plan_and_execute(left_target, right_target, true);
  }

  // 成员变量
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_exec_srv_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joint_state_mutex_;
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
