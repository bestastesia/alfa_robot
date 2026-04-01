// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * Alfa Robot ros2_control 硬件接口 - Franka 风格重构版
 * 通信层解耦到 CanBus 类，本类仅负责 ros2_control 接口适配
 */

#include "alfa_robot_hardware/alfa_robot_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

static bool hasInfinite(const std::vector<double> & vec)
{
  return std::any_of(vec.begin(), vec.end(),
    [](double v) { return !std::isfinite(v); });
}

bool AlfaRobotHW::isCanControlledJoint(const std::string & joint_name)
{
  return joint_name == "turn" ||
         joint_name == "leftjoint2" || joint_name == "leftjoint3" ||
         joint_name == "leftjoint4" || joint_name == "rightjoint2" ||
         joint_name == "rightjoint3" || joint_name == "rightjoint4";
}

bool AlfaRobotHW::isVelocityControlledJoint(const std::string & joint_name)
{
  return joint_name == "left_back" || joint_name == "left_forward" ||
         joint_name == "right_back" || joint_name == "right_forward";
}

bool AlfaRobotHW::isCanopenControlledJoint(const std::string & joint_name)
{
  return joint_name == "leftarmbase" || joint_name == "leftjoint1" ||
         joint_name == "rightarmbase" || joint_name == "rightjoint1" ||
         joint_name == "updown";
}

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Parse hardware_parameters into CanBusConfig
  can_bus_config_.can_interface_left = "can0";
  can_bus_config_.can_interface_right = "can1";
  can_bus_config_.can_interface_base = "can2";
  can_bus_config_.can_interface_canopen = "can3";
  can_bus_config_.max_speed_dps = 1800;
  can_bus_config_.canopen_profile_velocity = 50000;
  can_bus_config_.canopen_profile_accel = 50000;
  can_bus_config_.filter_cutoff_hz = 50.0;
  can_bus_config_.low_pass_filter_active = false;

  for (const auto & param : info_.hardware_parameters)
  {
    if (param.first == "can_interface_left")
    {
      can_bus_config_.can_interface_left = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "Left arm CAN interface: %s", can_bus_config_.can_interface_left.c_str());
    }
    else if (param.first == "can_interface_right")
    {
      can_bus_config_.can_interface_right = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "Right arm CAN interface: %s", can_bus_config_.can_interface_right.c_str());
    }
    else if (param.first == "can_interface_base")
    {
      can_bus_config_.can_interface_base = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "Base CAN interface: %s", can_bus_config_.can_interface_base.c_str());
    }
    else if (param.first == "can_interface_canopen")
    {
      can_bus_config_.can_interface_canopen = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "CANopen CAN interface: %s", can_bus_config_.can_interface_canopen.c_str());
    }
    else if (param.first == "max_speed_dps")
    {
      try
      {
        can_bus_config_.max_speed_dps = static_cast<uint16_t>(std::stoul(param.second));
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
          "max_speed_dps: %u", can_bus_config_.max_speed_dps);
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
          "Invalid max_speed_dps, using default 1440");
      }
    }
    else if (param.first == "canopen_profile_velocity")
    {
      try {
        can_bus_config_.canopen_profile_velocity = static_cast<uint32_t>(std::stoul(param.second));
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
          "CANopen profile velocity: %u pps", can_bus_config_.canopen_profile_velocity);
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
          "Invalid canopen_profile_velocity, using default");
      }
    }
    else if (param.first == "canopen_profile_accel")
    {
      try {
        can_bus_config_.canopen_profile_accel = static_cast<uint32_t>(std::stoul(param.second));
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
          "CANopen profile acceleration: %u pps²", can_bus_config_.canopen_profile_accel);
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
          "Invalid canopen_profile_accel, using default");
      }
    }
    else if (param.first == "filter_cutoff_hz")
    {
      try {
        can_bus_config_.filter_cutoff_hz = std::stod(param.second);
      } catch (const std::exception &) {}
    }
    else if (param.first == "enable_filter")
    {
      can_bus_config_.low_pass_filter_active = (param.second == "true");
    }
    else if (param.first == "use_safe_shutdown")
    {
      use_safe_shutdown_ = (param.second == "true");
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "Safe shutdown: %s", use_safe_shutdown_ ? "enabled" : "disabled");
    }
    else if (param.first.find("safe_position_") == 0)
    {
      std::string joint_name = param.first.substr(14);  // Remove "safe_position_" prefix
      try {
        safe_positions_[joint_name] = std::stod(param.second);
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
          "Safe position for '%s': %.3f rad", joint_name.c_str(), safe_positions_[joint_name]);
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
          "Invalid safe position value for '%s'", joint_name.c_str());
      }
    }
  }

  // Build RMD motor ID mapping (joint → motor_id + bus)
  static const std::map<std::string, RmdMotorInfo> kRmdMotorIds = {
    {"turn",        {1, RmdBus::BASE}},
    {"leftjoint2",  {1, RmdBus::LEFT}},  {"leftjoint3",  {2, RmdBus::LEFT}},  {"leftjoint4",  {3, RmdBus::LEFT}},
    {"rightjoint2", {4, RmdBus::RIGHT}}, {"rightjoint3", {5, RmdBus::RIGHT}}, {"rightjoint4", {6, RmdBus::RIGHT}},
  };

  size_t state_index = 0;
  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name)) { continue; }
    auto id_it = kRmdMotorIds.find(joint.name);
    if (id_it == kRmdMotorIds.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
        "Unknown RMD joint: %s", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    joint_to_motor_id_[joint.name] = id_it->second;
    joint_to_state_index_[joint.name] = state_index;
    joint_to_cmd_index_[joint.name] = state_index;

    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "RMD joint '%s' → motor ID %d, bus %d, index %zu",
      joint.name.c_str(), id_it->second.motor_id,
      static_cast<int>(id_it->second.bus), state_index);

    state_index++;
  }

  const size_t num_can_joints = joint_to_motor_id_.size();
  hw_positions_.resize(num_can_joints, 0.0);
  hw_velocities_.resize(num_can_joints, 0.0);
  hw_accelerations_.resize(num_can_joints, 0.0);
  previous_velocities_.resize(num_can_joints, 0.0);
  previous_positions_.resize(num_can_joints, 0.0);
  hw_position_commands_.resize(num_can_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Initialized %zu RMD joints", num_can_joints);

  // Build CANopen node ID mapping
  canopen_joint_to_node_id_["updown"] = 1;
  canopen_joint_to_node_id_["leftarmbase"] = 2;
  canopen_joint_to_node_id_["leftjoint1"] = 3;
  canopen_joint_to_node_id_["rightarmbase"] = 4;
  canopen_joint_to_node_id_["rightjoint1"] = 5;

  size_t canopen_idx = 0;
  for (const auto & joint : info_.joints)
  {
    if (!isCanopenControlledJoint(joint.name)) { continue; }
    canopen_joint_to_index_[joint.name] = canopen_idx;
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "CANopen joint '%s' → node %d, index %zu",
      joint.name.c_str(), canopen_joint_to_node_id_[joint.name], canopen_idx);
    ++canopen_idx;
  }

  const size_t num_canopen = canopen_joint_to_index_.size();
  canopen_positions_.resize(num_canopen, 0.0);
  canopen_velocities_.resize(num_canopen, 0.0);
  canopen_accelerations_.resize(num_canopen, 0.0);
  canopen_position_commands_.resize(num_canopen, 0.0);
  canopen_previous_positions_.resize(num_canopen, 0.0);
  canopen_previous_velocities_.resize(num_canopen, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Initialized %zu CANopen joints", num_canopen);

  // Legacy placeholder joints
  const size_t num_joints = info_.joints.size();
  hw_states_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);
  hw_velocities_legacy_.resize(num_joints, 0.0);
  hw_accelerations_legacy_.resize(num_joints, 0.0);
  hw_velocity_commands_legacy_.resize(num_joints, 0.0);
  previous_states_legacy_.resize(num_joints, 0.0);
  previous_velocities_legacy_.resize(num_joints, 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_bus_ = std::make_unique<CanBus>(can_bus_config_);
  if (!can_bus_->openInterfaces())
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to open CAN interfaces");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlfaRobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto & joint : info_.joints)
  {
    if (isCanControlledJoint(joint.name))
    {
      auto state_index_it = joint_to_state_index_.find(joint.name);
      if (state_index_it != joint_to_state_index_.end())
      {
        size_t idx = state_index_it->second;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[idx]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[idx]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_ACCELERATION, &hw_accelerations_[idx]));
      }
    }
    else if (isCanopenControlledJoint(joint.name))
    {
      auto idx_it = canopen_joint_to_index_.find(joint.name);
      if (idx_it != canopen_joint_to_index_.end())
      {
        size_t idx = idx_it->second;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &canopen_positions_[idx]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &canopen_velocities_[idx]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_ACCELERATION, &canopen_accelerations_[idx]));
      }
    }
    else
    {
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].name == joint.name)
        {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_legacy_[i]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_ACCELERATION, &hw_accelerations_legacy_[i]));
          break;
        }
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AlfaRobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const auto & joint : info_.joints)
  {
    if (isCanControlledJoint(joint.name))
    {
      auto cmd_index_it = joint_to_cmd_index_.find(joint.name);
      if (cmd_index_it != joint_to_cmd_index_.end())
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION,
          &hw_position_commands_[cmd_index_it->second]));
      }
    }
    else if (isCanopenControlledJoint(joint.name))
    {
      auto idx_it = canopen_joint_to_index_.find(joint.name);
      if (idx_it != canopen_joint_to_index_.end())
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION,
          &canopen_position_commands_[idx_it->second]));
      }
    }
    else
    {
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].name == joint.name)
        {
          if (isVelocityControlledJoint(joint.name))
          {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
              joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_legacy_[i]));
          }
          else
          {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
              joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
          }
          break;
        }
      }
    }
  }

  return command_interfaces;
}

void AlfaRobotHW::initializePositionCommands()
{
  if (first_position_update_)
  {
    for (size_t i = 0; i < hw_position_commands_.size(); ++i)
    {
      hw_position_commands_[i] = hw_positions_[i];
    }
    first_position_update_ = false;
  }

  if (canopen_first_position_update_)
  {
    for (size_t i = 0; i < canopen_position_commands_.size(); ++i)
    {
      canopen_position_commands_[i] = canopen_positions_[i];
    }
    canopen_first_position_update_ = false;
  }
}

hardware_interface::CallbackReturn AlfaRobotHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!can_bus_->enableMotors(joint_to_motor_id_, canopen_joint_to_node_id_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"), "Failed to enable motors");
    return CallbackReturn::ERROR;
  }

  // Initialize CANopen position commands from initial read
  for (const auto & pair : canopen_joint_to_node_id_)
  {
    if (can_bus_->enabledCanopenNodes().find(pair.second) == can_bus_->enabledCanopenNodes().end())
    {
      continue;
    }
    auto idx_it = canopen_joint_to_index_.find(pair.first);
    if (idx_it == canopen_joint_to_index_.end()) { continue; }

    double pos_m = 0.0;
    if (can_bus_->canopenReadPosition(pair.second, pos_m))
    {
      size_t idx = idx_it->second;
      // 减速比处理：leftarmbase 和 rightarmbase 有 3:1 减速机，电机位置需除以 3
      if (pair.first == "leftarmbase" || pair.first == "rightarmbase")
      {
        pos_m = pos_m / 3.0;
      }
      canopen_positions_[idx] = pos_m;
      canopen_position_commands_[idx] = pos_m;
      canopen_previous_positions_[idx] = pos_m;
    }
  }

  // First read to initialize RMD positions
  first_position_update_ = true;
  canopen_first_position_update_ = true;
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  // Capture turn joint's current raw position as software zero
  {
    auto it = joint_to_state_index_.find("turn");
    if (it != joint_to_state_index_.end())
    {
      size_t idx = it->second;
      turn_zero_offset_rad_ = hw_positions_[idx];
      hw_positions_[idx] = 0.0;
      previous_positions_[idx] = 0.0;
      hw_position_commands_[joint_to_cmd_index_["turn"]] = 0.0;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
        "Turn zero offset captured: %.4f rad (%.2f deg)",
        turn_zero_offset_rad_, turn_zero_offset_rad_ * 180.0 / M_PI);
    }
  }

  // Trajectory logging service node
  traj_log_node_ = rclcpp::Node::make_shared("traj_log_service");

  // SetBool: data=true → start (motor_id=6 for rightjoint4), data=false → stop
  traj_log_start_srv_ = traj_log_node_->create_service<std_srvs::srv::SetBool>(
    "traj_log/start_stop",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
           std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (req->data) {
        // Default to motor_id 6 (rightjoint4), can be extended later
        can_bus_->startTrajectoryLog(6);
        res->success = true;
        res->message = "Trajectory logging started for motor 6";
      } else {
        can_bus_->stopTrajectoryLog();
        res->success = true;
        res->message = "Trajectory logging stopped";
      }
    });

  // Trigger: dump CSV
  traj_log_dump_srv_ = traj_log_node_->create_service<std_srvs::srv::Trigger>(
    "traj_log/dump",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      can_bus_->stopTrajectoryLog();
      can_bus_->dumpTrajectoryLog("/tmp/traj_log.csv");
      res->success = true;
      res->message = "Trajectory log saved to /tmp/traj_log.csv";
    });

  traj_log_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  traj_log_executor_->add_node(traj_log_node_);
  traj_log_thread_ = std::thread([this]() { traj_log_executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Cleanup trajectory logging service
  if (traj_log_executor_)
  {
    traj_log_executor_->cancel();
  }
  if (traj_log_thread_.joinable())
  {
    traj_log_thread_.join();
  }
  traj_log_node_.reset();
  traj_log_start_srv_.reset();
  traj_log_dump_srv_.reset();
  traj_log_executor_.reset();

  if (use_safe_shutdown_ && !safe_positions_.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "Moving to safe position before deactivation...");

    if (!moveToSafePosition(5.0))
    {
      RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
        "Failed to reach safe position within timeout, proceeding with emergency stop");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Safe position reached");
    }
  }

  can_bus_->stopAll(joint_to_motor_id_, canopen_joint_to_node_id_);
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlfaRobotHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto state = can_bus_->readOnce(joint_to_motor_id_);
  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.0;

  // Update RMD buffers
  for (auto & [name, info] : joint_to_motor_id_)
  {
    auto it = state.rmd_positions.find(name);
    if (it == state.rmd_positions.end() || !it->second.valid) { continue; }
    size_t idx = joint_to_state_index_[name];
    double pos = it->second.position_rad;
    // Apply software zero offset for turn joint
    if (name == "turn")
    {
      pos -= turn_zero_offset_rad_;
    }
    hw_positions_[idx] = pos;
    if (dt > 0.0)
    {
      hw_velocities_[idx] = (pos - previous_positions_[idx]) / dt;
      hw_accelerations_[idx] = (hw_velocities_[idx] - previous_velocities_[idx]) / dt;
    }
    previous_positions_[idx] = pos;
    previous_velocities_[idx] = hw_velocities_[idx];
  }

  // Update CANopen buffers
  for (auto & [name, node_id] : canopen_joint_to_node_id_)
  {
    if (can_bus_->enabledCanopenNodes().find(node_id) == can_bus_->enabledCanopenNodes().end())
    {
      continue;
    }
    auto it = state.canopen_states.find(node_id);
    if (it == state.canopen_states.end() || !it->second.valid) { continue; }
    size_t idx = canopen_joint_to_index_[name];
    double pos = static_cast<double>(it->second.actual_position_pulses) / CanBus::kCanopenPulsesPerMeter;
    // 减速比处理：leftarmbase 和 rightarmbase 有 3:1 减速机，电机位置需除以 3
    if (name == "leftarmbase" || name == "rightarmbase")
    {
      pos = pos / 3.0;
    }
    canopen_positions_[idx] = pos;
    if (dt > 0.0)
    {
      canopen_velocities_[idx] = (pos - canopen_previous_positions_[idx]) / dt;
      canopen_accelerations_[idx] =
        (canopen_velocities_[idx] - canopen_previous_velocities_[idx]) / dt;
    }
    canopen_previous_positions_[idx] = pos;
    canopen_previous_velocities_[idx] = canopen_velocities_[idx];
  }

  // Franka-style: initialize commands in read()
  initializePositionCommands();

  // NaN safety
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    if (!std::isfinite(hw_positions_[i])) hw_positions_[i] = 0.0;
    if (!std::isfinite(hw_velocities_[i])) hw_velocities_[i] = 0.0;
    if (!std::isfinite(hw_accelerations_[i])) hw_accelerations_[i] = 0.0;
  }

  for (size_t i = 0; i < canopen_positions_.size(); ++i)
  {
    if (!std::isfinite(canopen_positions_[i])) canopen_positions_[i] = 0.0;
    if (!std::isfinite(canopen_velocities_[i])) canopen_velocities_[i] = 0.0;
    if (!std::isfinite(canopen_accelerations_[i])) canopen_accelerations_[i] = 0.0;
  }

  // Legacy placeholder joints
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (isCanControlledJoint(info_.joints[i].name) || isCanopenControlledJoint(info_.joints[i].name))
    {
      continue;
    }
    if (isVelocityControlledJoint(info_.joints[i].name))
    {
      hw_velocities_legacy_[i] = hw_velocity_commands_legacy_[i];
      if (dt > 0.0)
      {
        hw_states_[i] += hw_velocities_legacy_[i] * dt;
      }
      hw_accelerations_legacy_[i] = 0.0;
    }
    else
    {
      hw_states_[i] = hw_commands_[i];
      if (dt > 0.0)
      {
        hw_velocities_legacy_[i] = (hw_states_[i] - previous_states_legacy_[i]) / dt;
        hw_accelerations_legacy_[i] =
          (hw_velocities_legacy_[i] - previous_velocities_legacy_[i]) / dt;
      }
      previous_states_legacy_[i] = hw_states_[i];
      previous_velocities_legacy_[i] = hw_velocities_legacy_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlfaRobotHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Franka-style: NaN check returns ERROR
  if (hasInfinite(hw_position_commands_) || hasInfinite(canopen_position_commands_))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Franka-style: first_update guard
  if (first_position_update_ || canopen_first_position_update_)
  {
    return hardware_interface::return_type::OK;
  }

  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.005;

  // Assemble RMD commands
  std::map<std::string, double> rmd_cmds;
  for (auto & [name, info] : joint_to_motor_id_)
  {
    double cmd = hw_position_commands_[joint_to_cmd_index_[name]];
    // Apply software zero offset for turn joint
    if (name == "turn")
    {
      cmd += turn_zero_offset_rad_;
    }
    rmd_cmds[name] = cmd;
  }

  // Assemble CANopen commands
  std::map<uint8_t, double> canopen_cmds;
  for (auto & [name, node_id] : canopen_joint_to_node_id_)
  {
    if (can_bus_->enabledCanopenNodes().find(node_id) == can_bus_->enabledCanopenNodes().end())
    {
      continue;
    }
    double cmd = canopen_position_commands_[canopen_joint_to_index_[name]];
    // 减速比处理：leftarmbase 和 rightarmbase 有 3:1 减速机，关节位置需乘以 3
    if (name == "leftarmbase" || name == "rightarmbase")
    {
      cmd = cmd * 3.0;
    }
    canopen_cmds[node_id] = cmd;
  }

  can_bus_->writeOnce(rmd_cmds, canopen_cmds, dt);

  return hardware_interface::return_type::OK;
}

bool AlfaRobotHW::moveToSafePosition(double timeout_seconds)
{
  const double position_tolerance = 0.05;  // 0.05 rad (~2.86 degrees)
  const double control_period = 0.01;      // 10ms control loop
  const int max_iterations = static_cast<int>(timeout_seconds / control_period);

  // Set safe position commands
  for (const auto & [joint_name, safe_pos] : safe_positions_)
  {
    if (isCanControlledJoint(joint_name))
    {
      auto it = joint_to_cmd_index_.find(joint_name);
      if (it != joint_to_cmd_index_.end())
      {
        hw_position_commands_[it->second] = safe_pos;
      }
    }
    else if (isCanopenControlledJoint(joint_name))
    {
      auto it = canopen_joint_to_index_.find(joint_name);
      if (it != canopen_joint_to_index_.end())
      {
        canopen_position_commands_[it->second] = safe_pos;
      }
    }
  }

  // Control loop to reach safe position
  for (int i = 0; i < max_iterations; ++i)
  {
    // Read current positions
    read(rclcpp::Time(0), rclcpp::Duration::from_seconds(control_period));

    // Write commands
    write(rclcpp::Time(0), rclcpp::Duration::from_seconds(control_period));

    // Check if all joints reached safe position
    bool all_reached = true;
    for (const auto & [joint_name, safe_pos] : safe_positions_)
    {
      double current_pos = 0.0;
      bool found = false;

      if (isCanControlledJoint(joint_name))
      {
        auto it = joint_to_state_index_.find(joint_name);
        if (it != joint_to_state_index_.end())
        {
          current_pos = hw_positions_[it->second];
          found = true;
        }
      }
      else if (isCanopenControlledJoint(joint_name))
      {
        auto it = canopen_joint_to_index_.find(joint_name);
        if (it != canopen_joint_to_index_.end())
        {
          current_pos = canopen_positions_[it->second];
          found = true;
        }
      }

      if (found && std::abs(current_pos - safe_pos) > position_tolerance)
      {
        all_reached = false;
        break;
      }
    }

    if (all_reached)
    {
      return true;
    }

    // Sleep for control period
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(control_period * 1000)));
  }

  return false;  // Timeout
}

}  // namespace alfa_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)
