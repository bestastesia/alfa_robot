#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitPoseNode : public rclcpp::Node
{
public:
    InitPoseNode() : Node("init_pose_node")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&InitPoseNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        sent_ = false;

        RCLCPP_INFO(this->get_logger(), "InitPoseNode started");
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (sent_) return;

        geometry_msgs::msg::PoseWithCovarianceStamped init;

        init.header.stamp = this->now();
        init.header.frame_id = "map";   // AMCL global frame

        init.pose.pose = msg->pose.pose;

        // covariance（AMCL需要）
        for (int i = 0; i < 36; i++)
            init.pose.covariance[i] = 0.0;

        init.pose.covariance[0] = 0.25;
        init.pose.covariance[7] = 0.25;
        init.pose.covariance[35] = 0.1;

        rclcpp::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < 5; i++) {
            pub_->publish(init);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        sent_ = true;

        RCLCPP_INFO(this->get_logger(), "Initial pose sent to AMCL");
    }

    bool sent_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitPoseNode>());
    rclcpp::shutdown();
    return 0;
}