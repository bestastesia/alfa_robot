#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>  // 修复
#include <pcl/io/pcd_io.h>                 // PCL 用 .h 没问题
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp> // 修复
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream>
#include <filesystem>


class PcdToGridMapNode : public rclcpp::Node {
public:
  PcdToGridMapNode() : Node("pcd_to_grid_map_node") {
    // Declare parameters
    this->declare_parameter<std::string>("file_directory", "/home/");
    this->declare_parameter<std::string>("file_name", "map");
    this->declare_parameter<double>("thre_z_min", -0.9);
    this->declare_parameter<double>("thre_z_max", 0);
    this->declare_parameter<int>("flag_pass_through", 0);
    this->declare_parameter<double>("thre_radius", 0.5);
    this->declare_parameter<double>("map_resolution", 0.05);
    this->declare_parameter<int>("thres_point_count", 10);
    this->declare_parameter<std::string>("map_topic_name", "map");
    this->declare_parameter<double>("angle_min", -3.14159);
    this->declare_parameter<double>("angle_max", 3.14159);
    this->declare_parameter<double>("angle_increment", 0.0087);
    this->declare_parameter<double>("scan_time", 0.1);
    this->declare_parameter<double>("range_min", 0.1);
    this->declare_parameter<double>("range_max", 150.0);

    // Get parameters
    this->get_parameter("file_directory", file_directory_);
    this->get_parameter("file_name", file_name_);
    this->get_parameter("thre_z_min", thre_z_min_);
    this->get_parameter("thre_z_max", thre_z_max_);
    this->get_parameter("flag_pass_through", flag_pass_through_);
    this->get_parameter("thre_radius", thre_radius_);
    this->get_parameter("map_resolution", map_resolution_);
    this->get_parameter("thres_point_count", thres_point_count_);
    this->get_parameter("map_topic_name", map_topic_name_);
    this->get_parameter("angle_min", angle_min_);
    this->get_parameter("angle_max", angle_max_);
    this->get_parameter("angle_increment", angle_increment_);
    this->get_parameter("scan_time", scan_time_);
    this->get_parameter("range_min", range_min_);
    this->get_parameter("range_max", range_max_);

    std::string pcd_file = file_directory_ + file_name_ + ".pcd";

    // Load PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

    // Apply filters
    PassThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    RadiusOutlierFilter(cloud_after_PassThrough_, thre_radius_, thres_point_count_);

    // Convert to grid map
    SetMapTopicMsg(cloud_after_Radius_);

    // Publish map
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, 1);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PcdToGridMapNode::PublishMap, this)
    );
  }

private:
  void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in) {
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud_);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(thre_low, thre_high);
    passthrough.setNegative(flag_in);
    passthrough.filter(*cloud_after_PassThrough_);

    RCLCPP_INFO(this->get_logger(), "After PassThrough filter: %lu points", cloud_after_PassThrough_->points.size());
  }

  void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const double &radius, const int &thre_count) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
    radiusoutlier.setInputCloud(cloud);
    radiusoutlier.setRadiusSearch(radius);
    radiusoutlier.setMinNeighborsInRadius(thre_count);
    radiusoutlier.filter(*cloud_after_Radius_);

    RCLCPP_INFO(this->get_logger(), "After Radius filter: %lu points", cloud_after_Radius_->points.size());
  }

  void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Point cloud is empty!");
      return;
    }

    // Calculate bounding box
    double x_min = cloud->points[0].x;
    double x_max = cloud->points[0].x;
    double y_min = cloud->points[0].y;
    double y_max = cloud->points[0].y;

    for (const auto &point : cloud->points) {
      x_min = std::min(x_min, static_cast<double>(point.x));
      x_max = std::max(x_max, static_cast<double>(point.x));
      y_min = std::min(y_min, static_cast<double>(point.y));
      y_max = std::max(y_max, static_cast<double>(point.y));
    }

    // Set map metadata
    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = map_resolution_;
    map_msg_.info.origin.position.x = x_min;
    map_msg_.info.origin.position.y = y_min;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.info.width = static_cast<int>((x_max - x_min) / map_resolution_);
    map_msg_.info.height = static_cast<int>((y_max - y_min) / map_resolution_);

    // Initialize map data
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height, 0);

    // Fill map data
    for (const auto &point : cloud->points) {
      unsigned int i = static_cast<unsigned int>((point.x - x_min) / map_resolution_);
      unsigned int j = static_cast<unsigned int>((point.y - y_min) / map_resolution_);

      if (i < map_msg_.info.width && j < map_msg_.info.height) {
        map_msg_.data[i + j * map_msg_.info.width] = 100; // Mark as occupied
      }
    }

    RCLCPP_INFO(this->get_logger(), "Map created with size: %dx%d", map_msg_.info.width, map_msg_.info.height);

    // Save map to file
    SaveMapToFile(map_msg_);
  }

  void SaveMapToFile(const nav_msgs::msg::OccupancyGrid &map) {
    std::string result_dir = "/home/ar/fast_lio_ws/src/lio_2d_mapper/result/2d_map";
    
    // Create directory if it doesn't exist
    try {
      std::filesystem::create_directories(result_dir);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", e.what());
      return;
    }

    // Save map metadata
    std::ofstream metadata_file(result_dir + "/map_metadata.txt");
    if (metadata_file.is_open()) {
      metadata_file << "width: " << map.info.width << std::endl;
      metadata_file << "height: " << map.info.height << std::endl;
      metadata_file << "resolution: " << map.info.resolution << std::endl;
      metadata_file << "origin_x: " << map.info.origin.position.x << std::endl;
      metadata_file << "origin_y: " << map.info.origin.position.y << std::endl;
      metadata_file.close();
      RCLCPP_INFO(this->get_logger(), "Map metadata saved to: %s/map_metadata.txt", result_dir.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open metadata file for writing");
    }

    // Save map data as PGM
    std::string pgm_file = result_dir + "/map.pgm";
    std::ofstream pgm(pgm_file, std::ios::binary);
    if (pgm.is_open()) {
      // PGM header
      pgm << "P5\n";
      pgm << map.info.width << " " << map.info.height << "\n";
      pgm << "255\n";

      // Convert occupancy grid to PGM image
      for (unsigned int y = 0; y < map.info.height; ++y) {
        for (unsigned int x = 0; x < map.info.width; ++x) {
          unsigned int index = x + (map.info.height - 1 - y) * map.info.width;
          int value = map.data[index];
          unsigned char pixel = 255;
          
          if (value == 100) { // Occupied
            pixel = 0;
          } else if (value == 0) { // Free
            pixel = 255;
          } else { // Unknown
            pixel = 127;
          }
          
          pgm.put(pixel);
        }
      }
      
      pgm.close();
      RCLCPP_INFO(this->get_logger(), "Map saved to: %s", pgm_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file for writing");
    }

    // Save map data as YAML
    std::string yaml_file = result_dir + "/map.yaml";
    std::ofstream yaml(yaml_file);
    if (yaml.is_open()) {
      yaml << "image: map.pgm\n";
      yaml << "resolution: " << map.info.resolution << "\n";
      yaml << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0.0]\n";
      yaml << "negate: 0\n";
      yaml << "occupied_thresh: 0.65\n";
      yaml << "free_thresh: 0.196\n";
      yaml.close();
      RCLCPP_INFO(this->get_logger(), "YAML file saved to: %s", yaml_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing");
    }
  }

  void PublishMap() {
    map_msg_.header.stamp = this->get_clock()->now();
    map_publisher_->publish(map_msg_);
  }

  // Parameters
  std::string file_directory_;
  std::string file_name_;
  double thre_z_min_;
  double thre_z_max_;
  int flag_pass_through_;
  double thre_radius_;
  double map_resolution_;
  int thres_point_count_;
  std::string map_topic_name_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Map message
  nav_msgs::msg::OccupancyGrid map_msg_;

  // Publisher and timer
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToGridMapNode>());
  rclcpp::shutdown();
  return 0;
}