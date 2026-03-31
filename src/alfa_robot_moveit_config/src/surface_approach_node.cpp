#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <alfa_robot_moveit_config/srv/surface_approach.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <thread> // 引入多线程

class SurfaceApproachNode : public rclcpp::Node
{
public:
  SurfaceApproachNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
    : Node("surface_approach_node", options)
  {
    surface_approach_service_ = this->create_service<alfa_robot_moveit_config::srv::SurfaceApproach>(
      "surface_approach",
      std::bind(&SurfaceApproachNode::surfaceApproachCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Surface Approach Node initialized");
  }

  bool approachSurface(const std::string& arm_type, const geometry_msgs::msg::Point& center, const geometry_msgs::msg::Vector3& normal)
  {
    std::string group_name = (arm_type == "left") ? "left_arm" : "right_arm";
    RCLCPP_INFO(this->get_logger(), "Using arm: %s", group_name.c_str());

    // 【修改 1】创建一个专用的 Node 给 MoveIt，并放在独立线程中 Spin
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_node", node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner([&executor]() { executor.spin(); }); // 启动后台监听

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, group_name);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);

    // 获取规划参考系（通常是 base_link 或 world）
    std::string frame_id = move_group.getPlanningFrame();

    // ==========================================
    // 第1段：移动到离中心点10cm处，末端 X 轴平行于法向量
    // ==========================================
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = frame_id;
    target_pose1.header.stamp = this->get_clock()->now();

    Eigen::Vector3d center_eigen(center.x, center.y, center.z);
    Eigen::Vector3d normal_eigen(normal.x, normal.y, normal.z);
    normal_eigen.normalize(); 

    // 目标位置：中心点沿法向量退回10cm
    Eigen::Vector3d approach_position = center_eigen - normal_eigen * 0.1; 
    target_pose1.pose.position.x = approach_position.x();
    target_pose1.pose.position.y = approach_position.y();
    target_pose1.pose.position.z = approach_position.z();

    // 【修改 2】计算姿态：末端 X 轴指向法向量方向
    Eigen::Vector3d x_axis = normal_eigen;
    Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ(); 
    if (std::abs(x_axis.dot(z_axis)) > 0.9) {
      z_axis = Eigen::Vector3d::UnitY(); // 如果 X 和全局 Z 平行，改用 Y 作为参考
    }
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    z_axis = x_axis.cross(y_axis).normalized(); // 确保完全正交

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    Eigen::Quaterniond quaternion(rotation);

    target_pose1.pose.orientation.x = quaternion.x();
    target_pose1.pose.orientation.y = quaternion.y();
    target_pose1.pose.orientation.z = quaternion.z();
    target_pose1.pose.orientation.w = quaternion.w();

    move_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    
    if (move_group.plan(plan1) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan first trajectory");
      executor.cancel(); spinner.join();
      return false;
    }

    if (move_group.execute(plan1) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute first trajectory");
      executor.cancel(); spinner.join();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "First trajectory executed successfully");

    // ==========================================
    // 第2段：沿着末端 X 轴平移10cm (直线/笛卡尔路径)
    // ==========================================
    // 【修改 3】使用 computeCartesianPath 保证走直线
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose2 = target_pose1.pose;
    
    // 沿着 X 轴 (即 normal_eigen) 前进 10cm 到达中心点
    target_pose2.position.x += normal_eigen.x() * 0.1;
    target_pose2.position.y += normal_eigen.y() * 0.1;
    target_pose2.position.z += normal_eigen.z() * 0.1;
    waypoints.push_back(target_pose2);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // 1cm 解析度
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Cartesian path coverage: %.2f%%", fraction * 100.0);

    if (fraction < 0.9) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan complete Cartesian path for second trajectory");
      executor.cancel(); spinner.join();
      return false;
    }

    if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute second trajectory");
      executor.cancel(); spinner.join();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Second trajectory executed successfully");

    // 清理线程
    executor.cancel();
    spinner.join();
    return true;
  }

private:
  rclcpp::Service<alfa_robot_moveit_config::srv::SurfaceApproach>::SharedPtr surface_approach_service_;

  void surfaceApproachCallback(
    const std::shared_ptr<alfa_robot_moveit_config::srv::SurfaceApproach::Request> request,
    std::shared_ptr<alfa_robot_moveit_config::srv::SurfaceApproach::Response> response)
  {
    geometry_msgs::msg::Point center;
    center.x = request->center_x; center.y = request->center_y; center.z = request->center_z;
    geometry_msgs::msg::Vector3 normal;
    normal.x = request->normal_x; normal.y = request->normal_y; normal.z = request->normal_z;

    Eigen::Vector3d normal_eigen(normal.x, normal.y, normal.z);
    if (normal_eigen.norm() < 0.001) {
      response->success = false;
      response->message = "Invalid normal vector: magnitude is too small";
      return;
    }

    bool success = approachSurface(request->arm_type, center, normal);
    response->success = success;
    response->message = success ? "Successfully approached surface" : "Failed to approach surface";
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 继承 launch 文件或全局参数
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<SurfaceApproachNode>(node_options);
  
  if (argc == 8) {
    std::string arm_type = argv[1];
    geometry_msgs::msg::Point center;
    center.x = std::stod(argv[2]); center.y = std::stod(argv[3]); center.z = std::stod(argv[4]);
    geometry_msgs::msg::Vector3 normal;
    normal.x = std::stod(argv[5]); normal.y = std::stod(argv[6]); normal.z = std::stod(argv[7]);
    
    node->approachSurface(arm_type, center, normal);
  } else {
    RCLCPP_INFO(node->get_logger(), "Running in service mode. Waiting for requests...");
    rclcpp::spin(node);
  }
  
  rclcpp::shutdown();
  return 0;
}
