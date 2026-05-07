#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
#include <fstream>
#include <vector>
#include <string>

class OfflineMapPublisher : public rclcpp::Node
{
public:
    OfflineMapPublisher() : Node("offline_map_publisher")
    {
        // 声明参数
        this->declare_parameter<std::string>("map_path", "/home/ar/fast_lio_ws/src/lio_2d_mapper/maps/map.pgm");
        this->declare_parameter<std::string>("map_yaml_path", "/home/ar/fast_lio_ws/src/lio_2d_mapper/maps/map.yaml");
        this->declare_parameter<std::string>("map_topic", "/offline_map");
        this->declare_parameter<std::string>("map_frame", "map");

        // 获取参数
        std::string map_path = this->get_parameter("map_path").as_string();
        std::string map_yaml_path = this->get_parameter("map_yaml_path").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string map_frame = this->get_parameter("map_frame").as_string();

        // 读取PGM文件
        if (!readPGM(map_path, map_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read PGM file: %s", map_path.c_str());
            return;
        }

        // 读取YAML文件
        if (!readYAML(map_yaml_path, map_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read YAML file: %s", map_yaml_path.c_str());
            return;
        }

        // 设置地图帧
        map_.header.frame_id = map_frame;

        // 创建发布者
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, qos);

        // 发布地图
        map_.header.stamp = this->now();
        map_pub_->publish(map_);

        RCLCPP_INFO(this->get_logger(), "Offline map published successfully: %s", map_path.c_str());

        // 定期发布地图
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OfflineMapPublisher::publishMap, this)
        );
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_;

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_msg = map_;
        map_msg.header.stamp = this->now();
        map_pub_->publish(map_msg);
    }

    bool readPGM(const std::string& path, nav_msgs::msg::OccupancyGrid& map)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open PGM file: %s", path.c_str());
            return false;
        }

        // 读取PGM头部
        std::string magic;
        file >> magic;
        if (magic != "P5") {
            RCLCPP_ERROR(this->get_logger(), "Invalid PGM file format: %s", path.c_str());
            return false;
        }

        // 跳过注释
        char c;
        while (file.peek() == '#') {
            file.ignore(1024, '\n');
        }

        // 读取宽度和高度
        int width, height, max_val;
        file >> width >> height >> max_val;

        // 跳过换行符
        file.get(c);

        // 读取图像数据
        std::vector<uint8_t> image_data(width * height);
        file.read(reinterpret_cast<char*>(image_data.data()), width * height);

        // 转换为OccupancyGrid
        map.info.width = width;
        map.info.height = height;
        map.data.resize(width * height);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                int pgm_idx = (height - 1 - y) * width + x; // PGM是从下到上存储的

                uint8_t pixel = image_data[pgm_idx];
                if (pixel == 0) {
                    map.data[idx] = 100; // 障碍物
                } else if (pixel == 255) {
                    map.data[idx] = 0; // 自由空间
                } else {
                    map.data[idx] = -1; // 未知
                }
            }
        }

        return true;
    }

    bool readYAML(const std::string& path, nav_msgs::msg::OccupancyGrid& map)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Could not open YAML file: %s", path.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.find("resolution") != std::string::npos) {
                map.info.resolution = std::stof(line.substr(line.find(':') + 1));
            } else if (line.find("origin") != std::string::npos) {
                std::string origin = line.substr(line.find('[') + 1, line.find(']') - line.find('[') - 1);
                std::stringstream ss(origin);
                std::string token;
                std::getline(ss, token, ',');
                map.info.origin.position.x = std::stof(token);
                std::getline(ss, token, ',');
                map.info.origin.position.y = std::stof(token);
                map.info.origin.position.z = 0.0;
            }
        }

        map.info.origin.orientation.w = 1.0;

        return true;
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflineMapPublisher>());
    rclcpp::shutdown();
    return 0;
}