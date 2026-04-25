#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

class FovFilterNode : public rclcpp::Node
{
public:
  FovFilterNode() : Node("fov_filter_node")
  {
    // D455 RGB FOV: H=90°, V=65°
    constexpr double h_fov_deg = 90.0;
    constexpr double v_fov_deg = 65.0;
    tan_half_h_ = std::tan(h_fov_deg * 0.5 * M_PI / 180.0);  // tan(45°) = 1.0
    tan_half_v_ = std::tan(v_fov_deg * 0.5 * M_PI / 180.0);  // tan(32.5°) ≈ 0.6371

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pcd_for_cam", rclcpp::SensorDataQoS(),
      std::bind(&FovFilterNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_in_rgb", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "FOV filter ready: H=%.0f° V=%.0f°", h_fov_deg, v_fov_deg);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 找到 x, y, z 字段的偏移量
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto &field : msg->fields) {
      if (field.name == "x") x_off = field.offset;
      else if (field.name == "y") y_off = field.offset;
      else if (field.name == "z") z_off = field.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "PointCloud2 missing xyz fields");
      return;
    }

    const uint32_t point_step = msg->point_step;
    const uint8_t *data_ptr = msg->data.data();
    const size_t total_points = msg->width * msg->height;

    // 预分配输出
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.fields = msg->fields;
    out.point_step = point_step;
    out.height = 1;
    out.is_bigendian = msg->is_bigendian;
    out.is_dense = msg->is_dense;
    out.data.reserve(msg->data.size());

    uint32_t count = 0;
    for (size_t i = 0; i < total_points; ++i) {
      const uint8_t *p = data_ptr + i * point_step;
      float x, y, z;
      std::memcpy(&x, p + x_off, sizeof(float));
      std::memcpy(&y, p + y_off, sizeof(float));
      std::memcpy(&z, p + z_off, sizeof(float));

      // 坐标系: X前, Y左, Z上; 相机朝前(+X)
      // 点必须在相机前方，且在 FOV 立体角内
      if (x > 0.0f &&
          std::fabs(y) <= x * tan_half_h_ &&
          std::fabs(z) <= x * tan_half_v_) {
        out.data.insert(out.data.end(), p, p + point_step);
        ++count;
      }
    }

    out.width = count;
    out.row_step = count * point_step;
    pub_->publish(out);
  }

  double tan_half_h_;
  double tan_half_v_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FovFilterNode>());
  rclcpp::shutdown();
  return 0;
}
