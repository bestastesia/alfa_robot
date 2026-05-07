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
        log_odds_.resize(size_x_ * size_y_, 0.0f);

        // 20Hz定时发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Lio2DMapper::timerCallback, this)
        );
        
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered",
            10,
            std::bind(&Lio2DMapper::cloudCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/online_map", 
            rclcpp::QoS(1).reliable()  // 不需要transient_local
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&Lio2DMapper::odomCallback, this, std::placeholders::_1)
        );

        //grid_.resize(size_x_ * size_y_, 0.0);

        RCLCPP_INFO(this->get_logger(), "lio_2d_mapper started");

        save_map_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/save_map", 10,
            std::bind(&Lio2DMapper::saveMapCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "lio_2d_mapper 启动！等待 /save_map 触发保存...");
        


    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_; 

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_map_sub_; 

    /* 地图分辨率 */
    float resolution_ = 0.05;

    /* 地图范围 */
    float min_z_ = -0.9;
    float max_z_ = 0.5;

    /* 地图大小 */
    int size_x_ = 500;
    int size_y_ = 500;

    /* 地图概率 */
    std::vector<float> log_odds_;

    /* 地图原点 */
    float origin_x_ = 0;
    float origin_y_ = 0;

    /* 命中权重 */
    float log_odds_hit_ = 0.9;
    float log_odds_miss_ = -0.1;

    /* 地图概率最小值 */
    float log_odds_min_ = -2.0;

    /* 地图概率最大值 */
    float log_odds_max_ = 4.5;

    /* 机器人位置 */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
    float last_origin_x_ = 0.0f;
    float last_origin_y_ = 0.0f;

    float decay_factor_  =  0.99f;   // 慢衰减

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    /* 
    1.遍历点云里每个点的 x/y/z
    2.过滤掉太高 / 太低的点（只保留地面附近）
    3.把 3D 点映射到 2D 网格坐标
    4.在网格里标记 “这里有障碍物”
    5. 完成一帧 2D 地图更新
     */
    {
        for (auto& v : log_odds_) v *= decay_factor_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
        // 清空地图
        //std::fill(grid_.begin(), grid_.end(), 0);
        int rx = (robot_x_ - origin_x_) / resolution_ + size_x_ / 2;
        int ry = (robot_y_ - origin_y_) / resolution_ + size_y_ / 2;

        // 边界保护（很重要）
        if (rx < 0 || rx >= size_x_ || ry < 0 || ry >= size_y_)
            return;
    
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (*iter_z < min_z_ || *iter_z > max_z_)
                continue;
    
            int gx = (*iter_x - origin_x_) / resolution_ + size_x_ / 2;
            int gy = (*iter_y - origin_y_) / resolution_ + size_y_ / 2;
    
            if (gx < 0 || gx >= size_x_ || gy < 0 || gy >= size_y_)
                continue;
    
            // free space（射线）
            raytrace(rx, ry, gx, gy);

            // occupied（障碍物）
            int idx = gy * size_x_ + gx;

            log_odds_[idx] += log_odds_hit_;
            if (log_odds_[idx] > log_odds_max_)
                log_odds_[idx] = log_odds_max_;
            
        }
    
        // RCLCPP_INFO(this->get_logger(), "2D grid updated");

        //publishMap();

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }



    /* 发布地图 */
    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map;

        map.header.stamp = this->now();
        map.header.frame_id = "camera_init";

        map.info.resolution = resolution_;
        map.info.width = size_x_;
        map.info.height = size_y_;

        map.info.origin.position.x = -size_x_ * resolution_ / 2.0;
        map.info.origin.position.y = -size_y_ * resolution_ / 2.0;
        map.info.origin.position.z = 0.0;

        map.info.origin.orientation.w = 1.0;

        std::vector<int8_t> data(size_x_ * size_y_);

        for (size_t i = 0; i < log_odds_.size(); i++)
        {
            float p = 1.0 - 1.0 / (1.0 + std::exp(log_odds_[i]));
    
            if (p > 0.65)
                data[i] = 100;
            else if (p < 0.35)
                data[i] = 0;
            else
                data[i] = -1;
        }

        map.data = data;

        map_pub_->publish(map);
    }

    /* 射线追踪 */
    void raytrace(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);

        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;

        int err = dx - dy;

        while (true)
        {
            int idx = y0 * size_x_ + x0;

            // free space update
            log_odds_[idx] += log_odds_miss_;
            if (log_odds_[idx] < log_odds_min_)
                log_odds_[idx] = log_odds_min_;

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }


    void saveMapCallback(const std_msgs::msg::Empty::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "收到保存指令，正在保存地图...");

        std::string name = "map_2d";

        // 保存 PGM 图像
        FILE* f = fopen((name + ".pgm").c_str(), "w");
        fprintf(f, "P5\n%d %d\n255\n", size_x_, size_y_);

        for (int y = size_y_ - 1; y >= 0; y--) {
            for (int x = 0; x < size_x_; x++) {
                int idx = y * size_x_ + x;
                float p = 1.0f - 1.0f / (1.0f + std::exp(log_odds_[idx]));

                uint8_t c;
                if (p > 0.65)      c = 0;    // 障碍：黑
                else if (p < 0.35) c = 255;  // 自由：白
                else               c = 255;  // 未知：bai

                fwrite(&c, 1, 1, f);
            }
        }
        fclose(f);

        // 保存 YAML
        std::ofstream yaml(name + ".yaml");
        yaml << "image: " << name << ".pgm\n"
             << "resolution: " << resolution_ << "\n"
             << "origin: [" << (origin_x_ - size_x_*resolution_/2.0) << ", "
             << (origin_y_ - size_y_*resolution_/2.0) << ", 0.0]\n"
             << "negate: 0\n"
             << "occupied_thresh: 0.65\n"
             << "free_thresh: 0.35\n";
        yaml.close();

        RCLCPP_INFO(this->get_logger(), "地图保存成功：%s.pgm, %s.yaml", name.c_str(), name.c_str());
    }

    void timerCallback()
    {
        publishMap();
    }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lio2DMapper>());
    rclcpp::shutdown();
    return 0;
}