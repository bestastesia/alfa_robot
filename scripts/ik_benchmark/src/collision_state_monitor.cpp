#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <urdf/urdf/model.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

std::string readFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file.good()) {
        throw std::runtime_error("Cannot read: " + path);
    }
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::string loadCurrentUrdf()
{
    const std::string desc_share = ament_index_cpp::get_package_share_directory("alfa_robot_description");
    std::string urdf_path = desc_share + "/urdf/alfa_robot/alfa_robot.urdf";
    std::ifstream existing(urdf_path);
    if (existing.good()) {
        return readFile(urdf_path);
    }

    const std::string xacro_path = desc_share + "/urdf/alfa_robot.urdf.xacro";
    const std::string generated = "/tmp/alfa_collision_state_monitor.urdf";
    const int ret = std::system(("xacro " + xacro_path + " > " + generated).c_str());
    if (ret != 0) {
        throw std::runtime_error("xacro failed for " + xacro_path);
    }
    return readFile(generated);
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

std::string joinPairs(const std::vector<std::string>& pairs, size_t max_count = 8)
{
    std::ostringstream out;
    for (size_t i = 0; i < pairs.size() && i < max_count; ++i) {
        if (i > 0) {
            out << "\n";
        }
        out << pairs[i];
    }
    if (pairs.size() > max_count) {
        out << "\n... +" << (pairs.size() - max_count) << " more";
    }
    return out.str();
}

}  // namespace

class CollisionStateMonitor : public rclcpp::Node
{
public:
    CollisionStateMonitor()
        : Node("alfa_collision_state_monitor")
    {
        group_name_ = declare_parameter<std::string>("group_name", "");
        joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
        marker_topic_ = declare_parameter<std::string>("marker_topic", "/alfa_collision_markers");
        check_period_ = declare_parameter<double>("check_period", 0.2);
        verbose_ = declare_parameter<bool>("verbose", true);

        loadPlanningScene();

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                    joint_positions_[msg->name[i]] = msg->position[i];
                }
                have_state_ = true;
            });
        timer_ = create_wall_timer(std::chrono::duration<double>(check_period_), [this]() { checkAndPublish(); });

        RCLCPP_INFO(get_logger(), "collision monitor ready: joint_states=%s markers=%s group='%s'",
                    joint_states_topic_.c_str(), marker_topic_.c_str(), group_name_.c_str());
    }

private:
    void loadPlanningScene()
    {
        const std::string urdf_xml = loadCurrentUrdf();
        const std::string moveit_share = ament_index_cpp::get_package_share_directory("alfa_robot_moveit_config");
        const std::string srdf_xml = readFile(moveit_share + "/config/alfa_robot.srdf");

        auto urdf_model = std::make_shared<urdf::Model>();
        if (!urdf_model->initString(urdf_xml)) {
            throw std::runtime_error("Failed to parse URDF");
        }
        auto srdf_model = std::make_shared<srdf::Model>();
        if (!srdf_model->initString(*urdf_model, srdf_xml)) {
            throw std::runtime_error("Failed to parse SRDF");
        }

        robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
        const auto& variable_names = robot_model_->getVariableNames();
        model_variables_.insert(variable_names.begin(), variable_names.end());
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
        state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
        state_->setToDefaultValues();
    }

    void checkAndPublish()
    {
        if (!have_state_) {
            return;
        }

        for (const auto& [name, value] : joint_positions_) {
            if (model_variables_.count(name) > 0) {
                state_->setVariablePosition(name, value);
            }
        }
        state_->update();
        state_->updateCollisionBodyTransforms();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = group_name_;
        req.contacts = true;
        req.max_contacts = 50;
        req.max_contacts_per_pair = 1;
        req.verbose = false;
        planning_scene_->checkCollision(req, res, *state_, planning_scene_->getAllowedCollisionMatrix());

        std::vector<std::string> pairs;
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        for (const auto& contact_entry : res.contacts) {
            const auto& pair = contact_entry.first;
            pairs.push_back(pair.first + " <-> " + pair.second);
            for (const auto& contact : contact_entry.second) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "base_link";
                marker.header.stamp = now();
                marker.ns = "alfa_collision_contacts";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = contact.pos.x();
                marker.pose.position.y = contact.pos.y();
                marker.pose.position.z = contact.pos.z();
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.045;
                marker.scale.y = 0.045;
                marker.scale.z = 0.045;
                marker.color = makeColor(1.0f, 0.0f, 0.0f, 0.95f);
                marker.lifetime = rclcpp::Duration::from_seconds(check_period_ * 2.5);
                markers.markers.push_back(marker);
            }
        }

        visualization_msgs::msg::Marker text;
        text.header.frame_id = "base_link";
        text.header.stamp = now();
        text.ns = "alfa_collision_status";
        text.id = marker_id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = 0.0;
        text.pose.position.y = 0.0;
        text.pose.position.z = 1.8;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.08;
        text.color = res.collision ? makeColor(1.0f, 0.0f, 0.0f, 1.0f) : makeColor(0.0f, 1.0f, 0.0f, 1.0f);
        text.text = res.collision ? ("COLLISION\n" + joinPairs(pairs)) : "collision free";
        text.lifetime = rclcpp::Duration::from_seconds(check_period_ * 2.5);
        markers.markers.push_back(text);

        marker_pub_->publish(markers);

        if (verbose_ && res.collision) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "collision: %s", joinPairs(pairs, 4).c_str());
        }
    }

    std::string group_name_;
    std::string joint_states_topic_;
    std::string marker_topic_;
    double check_period_ = 0.2;
    bool verbose_ = true;
    bool have_state_ = false;
    std::map<std::string, double> joint_positions_;
    std::set<std::string> model_variables_;

    moveit::core::RobotModelPtr robot_model_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    std::unique_ptr<moveit::core::RobotState> state_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<CollisionStateMonitor>());
    } catch (const std::exception& e) {
        std::cerr << "collision_state_monitor failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
