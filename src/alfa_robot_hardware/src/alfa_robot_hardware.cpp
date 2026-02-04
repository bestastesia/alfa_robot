// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * Alfa Robot ros2_control 硬件接口 - Franka 风格
 * 协议：0x92 读取多圈角度，0xA4 多圈位置闭环控制，减速比 1:36
 * 启动前请执行: sudo ip link set can0 txqueuelen 256  (防止 ENOBUFS)
 */

namespace
{
constexpr unsigned int kCanInterFrameDelayUs = 150;
}

#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <unistd.h>

#include "alfa_robot_hardware/alfa_robot_hardware.hpp"
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
  return joint_name == "leftjoint2" || joint_name == "leftjoint3" ||
         joint_name == "leftjoint4" || joint_name == "rightjoint2" ||
         joint_name == "rightjoint3" || joint_name == "rightjoint4";
}

bool AlfaRobotHW::isVelocityControlledJoint(const std::string & joint_name)
{
  return joint_name == "left back" || joint_name == "left forward" ||
         joint_name == "right back" || joint_name == "right forward";
}

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  can_socket_ = -1;
  can_interface_ = "can0";
  max_speed_dps_ = 360;

  for (const auto & param : info_.hardware_parameters)
  {
    if (param.first == "can_interface")
    {
      can_interface_ = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "CAN interface set to: %s", can_interface_.c_str());
    }
    else if (param.first == "max_speed_dps")
    {
      try
      {
        max_speed_dps_ = static_cast<uint16_t>(std::stoul(param.second));
        RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "max_speed_dps set to: %u", max_speed_dps_);
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"), "Invalid max_speed_dps, using default 360");
      }
    }
  }

  uint8_t motor_id = 1;
  size_t state_index = 0;
  size_t cmd_index = 0;

  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }
    if (motor_id > 6)
    {
      RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
        "Too many CAN-controlled joints! Maximum is 6.");
      return CallbackReturn::ERROR;
    }

    joint_to_motor_id_[joint.name] = motor_id;
    joint_to_state_index_[joint.name] = state_index;
    joint_to_cmd_index_[joint.name] = cmd_index;

    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
      "CAN-controlled joint '%s' mapped to motor ID %d",
      joint.name.c_str(), motor_id);

    state_index++;
    cmd_index++;
    motor_id++;
  }

  const size_t num_can_joints = joint_to_motor_id_.size();
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Initialized %zu CAN-controlled joints", num_can_joints);

  hw_positions_.resize(num_can_joints, 0.0);
  hw_velocities_.resize(num_can_joints, 0.0);
  hw_accelerations_.resize(num_can_joints, 0.0);
  previous_velocities_.resize(num_can_joints, 0.0);
  previous_positions_.resize(num_can_joints, 0.0);
  hw_position_commands_.resize(num_can_joints, 0.0);

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
  if (!initCanInterface(can_interface_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to initialize CAN interface: %s", can_interface_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware configured successfully");
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
    else
    {
      // ========== TODO: 占位关节 - 待实现 ==========
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
            // TODO: turn, updown, leftarmbase, rightarmbase, leftjoint1, rightjoint1 - position control
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
  if (!first_position_update_)
  {
    return;
  }
  for (size_t i = 0; i < hw_position_commands_.size(); ++i)
  {
    hw_position_commands_[i] = hw_positions_[i];
  }
  first_position_update_ = false;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t run_cmd_data[7] = {0};
  for (const auto & joint_motor_pair : joint_to_motor_id_)
  {
    uint8_t motor_id = joint_motor_pair.second;
    sendMotorCommand(motor_id, 0x88, run_cmd_data);
    RCLCPP_DEBUG(rclcpp::get_logger("AlfaRobotHW"),
      "Sent run command to motor ID %d (joint: %s)",
      motor_id, joint_motor_pair.first.c_str());
  }

  usleep(10000);

  first_position_update_ = true;
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Hardware activated, CAN-controlled motors started");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint8_t close_cmd_data[7] = {0};
  for (const auto & joint_motor_pair : joint_to_motor_id_)
  {
    uint8_t motor_id = joint_motor_pair.second;
    sendMotorCommand(motor_id, 0x80, close_cmd_data);
    RCLCPP_DEBUG(rclcpp::get_logger("AlfaRobotHW"),
      "Sent close command to motor ID %d (joint: %s)",
      motor_id, joint_motor_pair.first.c_str());
  }

  usleep(10000);
  closeCanInterface();

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "Hardware deactivated, CAN-controlled motors stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlfaRobotHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::map<uint8_t, double> motor_positions;

  // Send 0x92 read requests to all CAN motors
  uint8_t read_cmd_data[7] = {0};
  for (const auto & joint_motor_pair : joint_to_motor_id_)
  {
    uint8_t motor_id = joint_motor_pair.second;
    sendMotorCommand(motor_id, 0x92, read_cmd_data);
  }

  // Receive replies (0x92 format: DATA[0]=0x92, DATA[1-7]=motorAngle 7 bytes)
  for (size_t i = 0; i < 6; ++i)
  {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;

    if (!receiveCanFrame(can_id, data, dlc))
    {
      break;
    }

    if (can_id >= 0x140 && can_id <= 0x140 + 6 && dlc >= 8)
    {
      uint8_t motor_id = static_cast<uint8_t>(can_id - 0x140);
      double position_rad;
      if (parseMotorAngleReply0x92(data, position_rad))
      {
        motor_positions[motor_id] = position_rad;
      }
    }
  }

  const double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.0;

  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }

    uint8_t motor_id = getMotorIdForJoint(joint.name);
    if (motor_id == 0)
    {
      continue;
    }

    auto state_index_it = joint_to_state_index_.find(joint.name);
    if (state_index_it == joint_to_state_index_.end())
    {
      continue;
    }

    size_t state_index = state_index_it->second;

    auto pos_it = motor_positions.find(motor_id);
    if (pos_it != motor_positions.end())
    {
      double new_position = pos_it->second;
      hw_positions_[state_index] = new_position;

      if (dt > 0.0)
      {
        hw_velocities_[state_index] = (new_position - previous_positions_[state_index]) / dt;
        hw_accelerations_[state_index] =
          (hw_velocities_[state_index] - previous_velocities_[state_index]) / dt;
      }

      previous_positions_[state_index] = new_position;
      previous_velocities_[state_index] = hw_velocities_[state_index];
    }
  }

  initializePositionCommands();

  // 确保 CAN 关节状态均为有限值，避免控制器因 NaN/Inf 报错
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    if (!std::isfinite(hw_positions_[i])) hw_positions_[i] = 0.0;
    if (!std::isfinite(hw_velocities_[i])) hw_velocities_[i] = 0.0;
    if (!std::isfinite(hw_accelerations_[i])) hw_accelerations_[i] = 0.0;
  }

  // ========== TODO: 占位关节 - 替换为实际硬件读取 ==========
  // turn, updown, leftarmbase, rightarmbase, leftjoint1, rightjoint1, left/right back, left/right forward
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (isCanControlledJoint(info_.joints[i].name))
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 若命令含 NaN/Inf，用当前读取位置替代，避免 controller update 报错
  if (hasInfinite(hw_position_commands_))
  {
    for (size_t i = 0; i < hw_position_commands_.size() && i < hw_positions_.size(); ++i)
    {
      if (!std::isfinite(hw_position_commands_[i]))
      {
        hw_position_commands_[i] = hw_positions_[i];
      }
    }
  }

  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }

    uint8_t motor_id = getMotorIdForJoint(joint.name);
    if (motor_id == 0)
    {
      continue;
    }

    auto cmd_index_it = joint_to_cmd_index_.find(joint.name);
    if (cmd_index_it == joint_to_cmd_index_.end())
    {
      continue;
    }

    double position_rad = hw_position_commands_[cmd_index_it->second];
    uint8_t frame_data[7];
    convertPositionToCanFormat0xA4(position_rad, frame_data);
    sendMotorCommand(motor_id, 0xA4, frame_data);
  }

  // ========== TODO: turn / velocity 关节写入逻辑 - 待实现 ==========

  return hardware_interface::return_type::OK;
}

bool AlfaRobotHW::initCanInterface(const std::string & interface)
{
  can_socket_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to create CAN socket: %s", strerror(errno));
    return false;
  }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to get CAN interface index for %s: %s",
      interface.c_str(), strerror(errno));
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to bind CAN socket: %s", strerror(errno));
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // 增大发送缓冲区，减轻 ENOBUFS (No buffer space available)
  const int sndbuf_size = 65536;
  if (setsockopt(can_socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Failed to set SO_SNDBUF: %s", strerror(errno));
  }

  int flags = fcntl(can_socket_, F_GETFL, 0);
  if (flags >= 0)
  {
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "CAN interface %s initialized successfully", interface.c_str());
  return true;
}

void AlfaRobotHW::closeCanInterface()
{
  if (can_socket_ >= 0)
  {
    close(can_socket_);
    can_socket_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "CAN interface closed");
  }
}

bool AlfaRobotHW::sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc)
{
  if (can_socket_ < 0)
  {
    return false;
  }

  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  ssize_t nbytes = ::write(can_socket_, &frame, sizeof(struct can_frame));
  if (nbytes < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
        "Failed to send CAN frame (ID: 0x%X): %s", can_id, strerror(errno));
    }
    return false;
  }

  if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame)))
  {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
      "Partial CAN frame sent: %zd bytes", nbytes);
    return false;
  }

  return true;
}

bool AlfaRobotHW::receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc)
{
  if (can_socket_ < 0)
  {
    return false;
  }

  struct can_frame frame;
  ssize_t nbytes = ::read(can_socket_, &frame, sizeof(struct can_frame));

  if (nbytes < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"),
        "Failed to receive CAN frame: %s", strerror(errno));
    }
    return false;
  }

  if (nbytes == static_cast<ssize_t>(sizeof(struct can_frame)))
  {
    can_id = frame.can_id & CAN_SFF_MASK;
    dlc = frame.can_dlc;
    memcpy(data, frame.data, dlc);
    return true;
  }

  return false;
}

void AlfaRobotHW::sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t * data)
{
  uint32_t can_id = 0x140 + motor_id;
  uint8_t frame_data[8];
  frame_data[0] = cmd_byte;
  if (data != nullptr)
  {
    memcpy(&frame_data[1], data, 7);
  }
  else
  {
    memset(&frame_data[1], 0, 7);
  }

  if (sendCanFrame(can_id, frame_data, 8))
  {
    usleep(kCanInterFrameDelayUs);
  }
}

uint8_t AlfaRobotHW::getMotorIdForJoint(const std::string & joint_name)
{
  auto it = joint_to_motor_id_.find(joint_name);
  if (it != joint_to_motor_id_.end())
  {
    return it->second;
  }
  return 0;
}

bool AlfaRobotHW::parseMotorAngleReply0x92(const uint8_t * data, double & position_rad)
{
  if (data[0] != 0x92)
  {
    return false;
  }

  // DATA[1-7] = motorAngle 7 bytes (little-endian), 0.01 deg/LSB
  // Reconstruct 56-bit signed value with sign extension from bit 55
  int64_t raw = static_cast<int64_t>(data[1]) |
    (static_cast<int64_t>(data[2]) << 8) |
    (static_cast<int64_t>(data[3]) << 16) |
    (static_cast<int64_t>(data[4]) << 24) |
    (static_cast<int64_t>(data[5]) << 32) |
    (static_cast<int64_t>(data[6]) << 40) |
    (static_cast<int64_t>(data[7]) << 48);

  if (data[7] & 0x80)
  {
    raw |= (static_cast<int64_t>(0xFFULL) << 56);
  }

  double angle_deg = static_cast<double>(raw) * 0.01;
  position_rad = angle_deg * M_PI / 180.0 / static_cast<double>(kGearRatio);
  return true;
}

void AlfaRobotHW::convertPositionToCanFormat0xA4(double position_rad, uint8_t * frame_data)
{
  // 0xA4: DATA[0]=0x00, DATA[1-2]=maxSpeed, DATA[3-6]=angleControl
  // angleControl: 0.01 deg/LSB at motor side
  double angle_deg = position_rad * 180.0 / M_PI;
  int32_t angle_control = static_cast<int32_t>(angle_deg * 100.0 * static_cast<double>(kGearRatio));

  frame_data[0] = 0x00;
  frame_data[1] = static_cast<uint8_t>(max_speed_dps_ & 0xFF);
  frame_data[2] = static_cast<uint8_t>((max_speed_dps_ >> 8) & 0xFF);
  frame_data[3] = static_cast<uint8_t>(angle_control & 0xFF);
  frame_data[4] = static_cast<uint8_t>((angle_control >> 8) & 0xFF);
  frame_data[5] = static_cast<uint8_t>((angle_control >> 16) & 0xFF);
  frame_data[6] = static_cast<uint8_t>((angle_control >> 24) & 0xFF);
}

}  // namespace alfa_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)
