#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LidarTFNode : public rclcpp::Node
{
public:
    LidarTFNode() : Node("lidar_tf_node"), tf_buffer_(this->get_clock())
    {
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("2d_base_frame", "2d_base_link");

        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        two_d_base_frame_ = this->get_parameter("2d_base_frame").as_string();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarTFNode::transformCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "lidar_tf_node started: Listening to %s → Publishing TF %s→%s",
            base_frame_.c_str(), odom_frame_.c_str(), two_d_base_frame_.c_str());
    }

private:
    void transformCallback()
    {
        try {
            // 获取 base_link 相对于 odom 的变换
            geometry_msgs::msg::TransformStamped base_transform;
            try {
                base_transform = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                // 如果失败，尝试获取 base_link 相对于 world 的变换
                try {
                    base_transform = tf_buffer_.lookupTransform("world", base_frame_, tf2::TimePointZero);
                    odom_frame_ = "world"; // 更新父坐标系为world
                } catch (tf2::TransformException &ex2) {
                    // 如果都失败，跳过这次更新
                    return;
                }
            }

            // 创建 2D 变换，父坐标系为 odom
            geometry_msgs::msg::TransformStamped t_msg;
            t_msg.header.stamp    = base_transform.header.stamp;
            t_msg.header.frame_id = odom_frame_;
            t_msg.child_frame_id  = two_d_base_frame_;

            // 2D 投影：x和y位置与 base_link 相同，z轴始终为0
            t_msg.transform.translation.x = base_transform.transform.translation.x;
            t_msg.transform.translation.y = base_transform.transform.translation.y;
            t_msg.transform.translation.z = 0.0;

            // 提取 base_link 的四元数
            tf2::Quaternion q(
                base_transform.transform.rotation.x,
                base_transform.transform.rotation.y,
                base_transform.transform.rotation.z,
                base_transform.transform.rotation.w);

            // 只提取 yaw，roll/pitch 置零
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            tf2::Quaternion q_2d;
            q_2d.setRPY(0.0, 0.0, yaw);

            // 只保留 yaw 旋转
            t_msg.transform.rotation.x = q_2d.x();
            t_msg.transform.rotation.y = q_2d.y();
            t_msg.transform.rotation.z = q_2d.z();
            t_msg.transform.rotation.w = q_2d.w();

            tf_broadcaster_->sendTransform(t_msg);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    std::string odom_frame_;
    std::string base_frame_;
    std::string two_d_base_frame_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTFNode>());
    rclcpp::shutdown();
    return 0;
}