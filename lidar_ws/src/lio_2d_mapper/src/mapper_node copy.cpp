#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include "std_msgs/msg/empty.hpp"

class Lio2DMapper : public rclcpp::Node
{
public:
    Lio2DMapper() : Node("lio_2d_mapper")
    {
        // 局部地图：8x8米，0.05分辨率 = 160x160格
        size_x_ = 160;
        size_y_ = 160;
        log_odds_.resize(size_x_ * size_y_, 0.0f);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&Lio2DMapper::cloudCallback, this, std::placeholders::_1)
        );

        // 实时地图用普通QoS
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/local_map", rclcpp::QoS(1).reliable()
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&Lio2DMapper::odomCallback, this, std::placeholders::_1)
        );

        // 离线地图保存（保留原有功能）
        // 离线地图需要单独的全局log_odds，这里暂时保留save功能用全局地图
        rclcpp::QoS qos_tl(rclcpp::KeepLast(1));
        qos_tl.transient_local().reliable();
        offline_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/online_map", qos_tl
        );

        save_map_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/save_map", 10,
            std::bind(&Lio2DMapper::saveMapCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "lio_2d_mapper started, local map: %dx%d @ %.2fm",
                    size_x_, size_y_, resolution_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr offline_map_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_map_sub_;

    float resolution_ = 0.05f;
    float min_z_ = -0.9f;
    float max_z_ = 0.5f;

    int size_x_ = 160;   // 8米 / 0.05
    int size_y_ = 160;

    std::vector<float> log_odds_;

    float log_odds_hit_  =  1.5f;
    float log_odds_miss_ = -0.1f;
    float log_odds_min_  = -2.0f;
    float log_odds_max_  =  5.0f;
    float decay_factor_  =  0.95f;  // 每帧衰减系数

    float max_raytrace_range_ = 6.0f;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 每帧先做时间衰减，动态障碍物会逐渐消失
        for (auto& v : log_odds_) {
            v *= decay_factor_;
        }

        // 地图原点跟着机器人走（rolling window）
        float origin_x = robot_y_ - (size_x_ * resolution_ / 2.0f);
        float origin_y = -(robot_x_) - (size_y_ * resolution_ / 2.0f); 

        // 机器人在局部地图中的格子坐标（始终在中心）
        int rx = size_x_ / 2;
        int ry = size_y_ / 2;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (*iter_z < min_z_ || *iter_z > max_z_)
                continue;

            // 过滤无效点（inf/nan）
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y))
                continue;

            // 计算点到机器人的距离，太远的不做射线追踪
            float dx = *iter_x - robot_x_;
            float dy = *iter_y - robot_y_;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist > max_raytrace_range_)
                continue;

            int gx = static_cast<int>((*iter_y - origin_x) / resolution_);
            int gy = static_cast<int>((-(*iter_x) - origin_y) / resolution_);

            if (gx < 0 || gx >= size_x_ || gy < 0 || gy >= size_y_)
                continue;

            // 射线追踪标记free space
            raytrace(rx, ry, gx, gy);

            // 标记障碍物
            int idx = gy * size_x_ + gx;
            log_odds_[idx] += log_odds_hit_;
            if (log_odds_[idx] > log_odds_max_)
                log_odds_[idx] = log_odds_max_;
        }

        publishLocalMap(origin_x, origin_y, msg->header.stamp);
    }

    void publishLocalMap(float origin_x, float origin_y,
                         rclcpp::Time stamp)
    {
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = stamp;
        map.header.frame_id = "map";   // 必须是map

        map.info.resolution = resolution_;
        map.info.width  = size_x_;
        map.info.height = size_y_;
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.data.resize(size_x_ * size_y_);
        for (size_t i = 0; i < log_odds_.size(); i++)
        {
            float p = 1.0f - 1.0f / (1.0f + std::exp(log_odds_[i]));
            if (p > 0.65f)       map.data[i] = 100;
            else if (p < 0.35f)  map.data[i] = 0;
            else                 map.data[i] = -1;
        }

        map_pub_->publish(map);
    }

    void raytrace(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            // 到达终点就停，终点由hit单独处理
            if (x0 == x1 && y0 == y1) break;

            int idx = y0 * size_x_ + x0;
            if (idx >= 0 && idx < (int)log_odds_.size()) {
                if (log_odds_[idx] < 1.0f) {  // 只更新非障碍格子
                    log_odds_[idx] += log_odds_miss_;
                    if (log_odds_[idx] < log_odds_min_)
                        log_odds_[idx] = log_odds_min_;
                }
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    void saveMapCallback(const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "保存地图...");
        // 保留你原有的saveMap逻辑
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lio2DMapper>());
    rclcpp::shutdown();
    return 0;
}