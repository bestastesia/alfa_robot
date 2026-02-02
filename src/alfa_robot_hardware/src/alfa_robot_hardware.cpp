// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>

#include "alfa_robot_hardware/alfa_robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{
bool AlfaRobotHW::isCanControlledJoint(const std::string & joint_name)
{
  // 仅以下 6 个关节为 CAN 控制，使用精确匹配避免前缀/相似名误判（如 leftjoint21）
  return joint_name == "leftjoint2" || joint_name == "leftjoint3" ||
         joint_name == "leftjoint4" || joint_name == "rightjoint2" ||
         joint_name == "rightjoint3" || joint_name == "rightjoint4";
}

bool AlfaRobotHW::isVelocityControlledJoint(const std::string & joint_name)
{
  // 底盘轮子：velocity 命令接口，需在 export_command_interfaces 中导出 velocity
  return joint_name == "left back" || joint_name == "left forward" ||
         joint_name == "right back" || joint_name == "right forward";
}

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
  // 调用父类的on_init方法
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize CAN socket
  can_socket_ = -1;
  
  // Read CAN interface parameter 
  can_interface_ = "can0";  // Default
  for (const auto & param : info_.hardware_parameters)
  {
    if (param.first == "can_interface")
    {
      can_interface_ = param.second;
      RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "CAN interface set to: %s", can_interface_.c_str());
    }
  }
  
  // Initialize mappings only for CAN-controlled joints (leftjoint2-4, rightjoint2-4)
  uint8_t motor_id = 1;
  size_t state_index = 0;
  size_t cmd_index = 0;
  
  for (const auto & joint : info_.joints)
  {
    // Only process CAN-controlled joints
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }
    // 仅支持 6 个 CAN 关节，超过则报错
    if (motor_id > 6)
    {
      RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"), 
                   "Too many CAN-controlled joints! Maximum is 6.");
      return CallbackReturn::ERROR;
    }
    
    // Assign motor ID to joint
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
   // 输出初始化信息
  const size_t num_can_joints = joint_to_motor_id_.size();
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
              "Initialized %zu CAN-controlled joints", num_can_joints);
  
  // 存放CAN控制的关节状态向量
  hw_positions_.resize(num_can_joints, 0.0);
  hw_velocities_.resize(num_can_joints, 0.0);
  hw_accelerations_.resize(num_can_joints, 0.0);
  previous_velocities_.resize(num_can_joints, 0.0);
  
  // 存放CAN控制的关节命令向量 只写了位置控制
  hw_position_commands_.resize(num_can_joints, 0.0);
  
  // 所有关节的状态/命令向量（非 CAN 关节用于伪造 velocity/acceleration 接口，初始 0 避免 NaN）
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
  // Initialize CAN interface
  if (!initCanInterface(can_interface_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"), 
                 "Failed to initialize CAN interface: %s", can_interface_.c_str());
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
              "Hardware configured successfully");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlfaRobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (const auto & joint : info_.joints)
  {
    if (isCanControlledJoint(joint.name))
    {
      // 输出CAN控制关节的状态接口
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
      // 非 CAN 关节也需导出 position / velocity / acceleration，否则 joint_state_broadcaster 等会报 missing state interfaces
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
      // Export position command interface for CAN-controlled joints
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
      // 非 CAN 关节：轮子用 velocity 接口，其余用 position 接口（与 URDF/forward_velocity_controller 一致）
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

hardware_interface::CallbackReturn AlfaRobotHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send motor run command (0x88) to all CAN-controlled motors
  uint8_t run_cmd_data[7] = {0};
  for (const auto & joint_motor_pair : joint_to_motor_id_)
  {
    uint8_t motor_id = joint_motor_pair.second;
    sendMotorCommand(motor_id, 0x88, run_cmd_data);
    RCLCPP_DEBUG(rclcpp::get_logger("AlfaRobotHW"), 
                 "Sent run command to motor ID %d (joint: %s)", 
                 motor_id, joint_motor_pair.first.c_str());
  }
  
  // Small delay to allow motors to respond
  usleep(10000);  // 10ms
  
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
              "Hardware activated, CAN-controlled motors started");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send motor close command (0x80) to all CAN-controlled motors
  uint8_t close_cmd_data[7] = {0};
  for (const auto & joint_motor_pair : joint_to_motor_id_)
  {
    uint8_t motor_id = joint_motor_pair.second;
    sendMotorCommand(motor_id, 0x80, close_cmd_data);
    RCLCPP_DEBUG(rclcpp::get_logger("AlfaRobotHW"), 
                 "Sent close command to motor ID %d (joint: %s)", 
                 motor_id, joint_motor_pair.first.c_str());
  }
  
  // Small delay to allow motors to respond
  usleep(10000);  // 10ms
  
  // Close CAN interface
  closeCanInterface();
  
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
              "Hardware deactivated, CAN-controlled motors stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlfaRobotHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Receive CAN frames and update states for CAN-controlled motors only
  std::map<uint8_t, std::pair<double, double>> motor_states;  // motor_id -> {position, velocity}
  
  // Try to receive multiple frames (up to 6 motors)
  for (size_t i = 0; i < 6; ++i)
  {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    
    if (!receiveCanFrame(can_id, data, dlc))
    {
      break;  // No more frames available
    }
    
    // Extract motor ID from CAN ID (0x140 + motor_id)
    if (can_id >= 0x140 && can_id <= 0x140 + 6)
    {
      uint8_t motor_id = static_cast<uint8_t>(can_id - 0x140);
      // Parse status2 reply (command replies also use status2 format)
      double position, velocity;
      if (parseMotorStatus2(data, position, velocity))
      {
        motor_states[motor_id] = std::make_pair(position, velocity);
      }
    }
  }
  
  // Update state vectors based on motor states (only for CAN-controlled joints)
  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }
    
    // Get motor ID and state index for this joint
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
    
    // Update states if we received data for this motor
    auto state_it = motor_states.find(motor_id);
    if (state_it != motor_states.end())
    {
      double new_velocity = state_it->second.second;
      
      // Update position and velocity
      hw_positions_[state_index] = state_it->second.first;
      hw_velocities_[state_index] = new_velocity;
      
      // Calculate acceleration from velocity difference
      if (period.nanoseconds() > 0)
      {
        double dt = period.seconds();
        if (dt > 0.0)
        {
          hw_accelerations_[state_index] = 
            (new_velocity - previous_velocities_[state_index]) / dt;
        }
      }
      
      previous_velocities_[state_index] = new_velocity;
    }
  }

  // 非 CAN 关节：伪造 position/velocity/acceleration 以满足控制器对 state 接口的要求
  const double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.0;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (isCanControlledJoint(info_.joints[i].name))
    {
      continue;
    }
    if (isVelocityControlledJoint(info_.joints[i].name))
    {
      // 速度控制关节：位置积分，速度=命令，加速度伪造为 0
      hw_velocities_legacy_[i] = hw_velocity_commands_legacy_[i];
      if (dt > 0.0)
      {
        hw_states_[i] += hw_velocities_legacy_[i] * dt;
      }
      hw_accelerations_legacy_[i] = 0.0;
    }
    else
    {
      // 位置控制关节：状态跟随命令，velocity/acceleration 用数值微分伪造
      hw_states_[i] = hw_commands_[i];
      if (dt > 0.0)
      {
        hw_velocities_legacy_[i] =
          (hw_states_[i] - previous_states_legacy_[i]) / dt;
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
  // Send control commands only to CAN-controlled joints (leftjoint2-4, rightjoint2-4)
  for (const auto & joint : info_.joints)
  {
    if (!isCanControlledJoint(joint.name))
    {
      continue;
    }
    
    // Get motor ID and command index for this joint
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
    
    // 单圈位置闭环控制命令 2 (0xA6): DATA[1]=spinDirection, DATA[2-3]=maxSpeed(LE), DATA[4-7]=angleControl(LE), 0-360°->0-1296000
    double position_rad = hw_position_commands_[cmd_index_it->second];
    uint32_t angle_control;
    convertPositionToCanFormat(position_rad, angle_control);
    const uint8_t spin_direction = (position_rad < 0.0) ? 0x10u : 0x00u;  // 负角度反转
    const uint16_t max_speed_dps = 360;  // 1 dps/LSB

    uint8_t frame_data[7] = {0};
    frame_data[0] = spin_direction;
    frame_data[1] = static_cast<uint8_t>(max_speed_dps & 0xFF);
    frame_data[2] = static_cast<uint8_t>((max_speed_dps >> 8) & 0xFF);
    frame_data[3] = static_cast<uint8_t>(angle_control & 0xFF);
    frame_data[4] = static_cast<uint8_t>((angle_control >> 8) & 0xFF);
    frame_data[5] = static_cast<uint8_t>((angle_control >> 16) & 0xFF);
    frame_data[6] = static_cast<uint8_t>((angle_control >> 24) & 0xFF);
    sendMotorCommand(motor_id, 0xA6, frame_data);
  }

  return hardware_interface::return_type::OK;
}

// CAN communication helper functions
bool AlfaRobotHW::initCanInterface(const std::string & interface)
{
  // Create CAN socket
  can_socket_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"), 
                 "Failed to create CAN socket: %s", strerror(errno));
    return false;
  }
  
  // Get interface index
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
  
  // Bind socket to CAN interface
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
  
  // Set non-blocking mode for 1000Hz operation
  int flags = fcntl(can_socket_, F_GETFL, 0);
  if (flags < 0 || fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"), 
                "Failed to set non-blocking mode: %s", strerror(errno));
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
  
  // Use ::write to explicitly call POSIX write() function, not the class method
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
  
  if (nbytes != sizeof(struct can_frame))
  {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"), 
                "Partial CAN frame sent: %zd bytes instead of %zu", 
                nbytes, sizeof(struct can_frame));
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
  // Use ::read to explicitly call POSIX read() function, not the class method
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
  
  if (nbytes == sizeof(struct can_frame))
  {
    can_id = frame.can_id & CAN_SFF_MASK;  // Extract standard frame ID
    dlc = frame.can_dlc;
    memcpy(data, frame.data, dlc);
    return true;
  }
  
  if (nbytes > 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("AlfaRobotHW"), 
                "Partial CAN frame received: %zd bytes instead of %zu", 
                nbytes, sizeof(struct can_frame));
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
  
  sendCanFrame(can_id, frame_data, 8);
}

uint8_t AlfaRobotHW::getMotorIdForJoint(const std::string & joint_name)
{
  auto it = joint_to_motor_id_.find(joint_name);
  if (it != joint_to_motor_id_.end())
  {
    return it->second;
  }
  return 0;  // Invalid motor ID
}

void AlfaRobotHW::convertPositionToCanFormat(double position_rad, uint32_t & angle_control)
{
  // 单圈位置控制 2：0-360° 对应 0-1296000 (1°=3600 LSB)，角度归一化到 [0,360)，再除以减速比 36 避免转太多圈
  double angle_deg = std::fmod(position_rad * 180.0 / M_PI, 360.0);
  if (angle_deg < 0.0) angle_deg += 360.0;
  angle_control = static_cast<uint32_t>(angle_deg * 3600.0 / 36.0);
  if (angle_control > 36000u) angle_control = 36000u;  // 1296000/36
}

bool AlfaRobotHW::parseMotorStatus2(const uint8_t * data, double & position, double & velocity)
{
  constexpr int GEAR_RATIO = 36;
  // 0x92: 多圈绝对值，DATA[4-7]=32bit 多圈绝对值(电机圈数，小端)，输出角 = 电机圈数/36 * 2π
  if (data[0] == 0x92)
  {
    int32_t multi_turn = static_cast<int32_t>(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
    position = static_cast<double>(multi_turn) / GEAR_RATIO * 2.0 * M_PI;  // 电机圈 -> 输出弧度
    velocity = 0.0;  // 0x92 回复通常无速度，可后续按协议补充
    return true;
  }
  // status2: DATA[4-5]=speed(motor DPS), DATA[6-7]=encoder(motor side)
  int16_t speed_dps = static_cast<int16_t>(data[4] | (data[5] << 8));
  velocity = static_cast<double>(speed_dps) / GEAR_RATIO * M_PI / 180.0;
  uint16_t encoder = static_cast<uint16_t>(data[6] | (data[7] << 8));
  position = static_cast<double>(encoder) * 2.0 * M_PI / 65535.0 / GEAR_RATIO;
  return true;
}

}  // namespace alfa_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)
