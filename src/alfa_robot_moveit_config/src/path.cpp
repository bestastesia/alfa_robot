#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Dense>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 检查参数数量 (1个程序名 + 1个规划组名 + 3个位置 + 4个四元数 = 9)
    if (argc < 9) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "用法: ros2 run <包名> single_target_quat_node <group_name> <x> <y> <z> <qx> <qy> <qz> <qw>");
        return 1;
    }
    
    // 解析命令行参数
    std::string group_name = argv[1];
    double target_x = std::stod(argv[2]);
    double target_y = std::stod(argv[3]);
    double target_z = std::stod(argv[4]);
    double qx = std::stod(argv[5]);
    double qy = std::stod(argv[6]);
    double qz = std::stod(argv[7]);
    double qw = std::stod(argv[8]);

    // 初始化 ROS 2 节点
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("single_target_quat_node", node_options);
    
    // 创建轨迹发布器
    auto trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory", 10);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // 初始化 MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);

    // ================= 1. 规范化四元数 (良好的工程习惯) =================
    // 手动输入的四元数可能存在微小的精度误差导致平方和不严格等于1，MoveIt对此比较敏感
    Eigen::Quaterniond q(qw, qx, qy, qz); // Eigen 的构造函数顺序是 (w, x, y, z)
    q.normalize(); 

    // ================= 2. 组装目标 Pose =================
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    // 输出调试信息
    RCLCPP_INFO(node->get_logger(), "----------------------------------------");
    RCLCPP_INFO(node->get_logger(), "请求的目标位置: X=%.3f, Y=%.3f, Z=%.3f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "归一化后的四元数 (x,y,z,w): [%.4f, %.4f, %.4f, %.4f]", q.x(), q.y(), q.z(), q.w());
    RCLCPP_INFO(node->get_logger(), "----------------------------------------");
    
    // ================= 2.5 强制对齐规划基准 (解决 RViz 与代码不一致的核心) =================
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    // 1. 设置当前的起始状态为机器人的真实当前状态
    move_group.setStartStateToCurrentState();

    // 2. 强制指定参考坐标系 (非常重要！)
    // 请将其改为你在 RViz -> Global Options -> Fixed Frame 中看到的那个名字
    // 通常是 "world", "base_link", 或 "base"
    std::string reference_frame = "base_link"; 
    move_group.setPoseReferenceFrame(reference_frame);

    // 3. (可选) 强制指定末端执行器连杆
    // 如果你知道 RViz 里拖拽的是哪个连杆，在这里显式指定它
     std::string eef_link = "left_ee_link";
     move_group.setEndEffectorLink(eef_link);

    // 输出基准信息供排查
    RCLCPP_INFO(node->get_logger(), "【检查对齐】规划参考坐标系: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "【检查对齐】末端执行器连杆: %s", move_group.getEndEffectorLink().c_str());
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setPlanningTime(5.0);
    // ================= 3. 规划与执行 =================
    //move_group.setPoseTarget(target_pose);
    move_group.setApproximateJointValueTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(node->get_logger(), "正在调用规划器...");
    
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {        // 打印轨迹包含的路点数量
        size_t point_count = my_plan.trajectory_.joint_trajectory.points.size();
        RCLCPP_INFO(node->get_logger(), "规划成功！生成的轨迹共包含 %zu 个路点，开始发布...", point_count);

        // 发布轨迹到 trajectory_executor 节点
        trajectory_publisher->publish(my_plan.trajectory_.joint_trajectory);
        RCLCPP_INFO(node->get_logger(), "轨迹已发布到 /joint_trajectory 话题");
        
        // 等待轨迹发布完成
        rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败 (Invalid Goal)！");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
    // 清理资源
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
