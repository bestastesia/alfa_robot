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
  // Only leftjoint2, leftjoint3, leftjoint4, rightjoint2, rightjoint3, rightjoint4
  return joint_name.find("leftjoint2") != std::string::npos ||
         joint_name.find("leftjoint3") != std::string::npos ||
         joint_name.find("leftjoint4") != std::string::npos ||
         joint_name.find("rightjoint2") != std::string::npos ||
         joint_name.find("rightjoint3") != std::string::npos ||
         joint_name.find("rightjoint4") != std::string::npos;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
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
    
    // Assign motor ID to joint
    joint_to_motor_id_[joint.name] = motor_id;
    joint_to_state_index_[joint.name] = state_index;
    joint_to_cmd_index_[joint.name] = cmd_index;
    
    RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
                "CAN-controlled joint '%s' mapped to motor ID %d", 
                joint.name.c_str(), motor_id);
    
    motor_id++;
    state_index++;
    cmd_index++;
    
    if (motor_id > 6)
    {
      RCLCPP_ERROR(rclcpp::get_logger("AlfaRobotHW"), 
                   "Too many CAN-controlled joints! Maximum is 6.");
      return CallbackReturn::ERROR;
    }
  }
  
  const size_t num_can_joints = joint_to_motor_id_.size();
  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), 
              "Initialized %zu CAN-controlled joints", num_can_joints);
  
  // Initialize state vectors (6 CAN-controlled joints, 3 states each)
  hw_positions_.resize(num_can_joints, 0.0);
  hw_velocities_.resize(num_can_joints, 0.0);
  hw_accelerations_.resize(num_can_joints, 0.0);
  previous_velocities_.resize(num_can_joints, 0.0);
  
  // Initialize command vectors (position control for these 6 joints)
  hw_position_commands_.resize(num_can_joints, 0.0);
  
  // Legacy vectors (for all joints, including non-CAN controlled ones)
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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
      // Export position, velocity, and acceleration interfaces for CAN-controlled joints
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
      // For non-CAN controlled joints, use legacy state interface
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].name == joint.name)
        {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
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
      // For non-CAN controlled joints, use legacy command interface
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        if (info_.joints[i].name == joint.name)
        {
          command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
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
    
    // Position control command (0xA3): Multi-turn position closed-loop control command 1
    double position_rad = hw_position_commands_[cmd_index_it->second];
    int32_t angle_control;
    convertPositionToCanFormat(position_rad, angle_control);
    
    // Construct CAN frame: DATA[0]=0xA3, DATA[4-7]=angleControl (little-endian)
    uint8_t frame_data[8] = {0};
    frame_data[1] = 0;  // DATA[1] = NULL
    frame_data[2] = 0;  // DATA[2] = NULL
    frame_data[3] = 0;  // DATA[3] = NULL
    frame_data[4] = static_cast<uint8_t>(angle_control & 0xFF);
    frame_data[5] = static_cast<uint8_t>((angle_control >> 8) & 0xFF);
    frame_data[6] = static_cast<uint8_t>((angle_control >> 16) & 0xFF);
    frame_data[7] = static_cast<uint8_t>((angle_control >> 24) & 0xFF);
    
    sendMotorCommand(motor_id, 0xA3, &frame_data[1]);
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

void AlfaRobotHW::convertPositionToCanFormat(double position_rad, int32_t & angle_control)
{
  // Convert rad to 0.01degree: angleControl = position_rad * 18000.0 / PI
  angle_control = static_cast<int32_t>(position_rad * 18000.0 / M_PI);
}

bool AlfaRobotHW::parseMotorStatus2(const uint8_t * data, double & position, double & velocity)
{
  // Parse status2 reply: DATA[0]=cmd, DATA[1]=temp, DATA[2-3]=iq/power, 
  // DATA[4-5]=speed (int16_t, 1 DPS/LSB), DATA[6-7]=encoder (uint16_t)
  
  // Extract speed (int16_t, little-endian, 1 DPS/LSB)
  int16_t speed_dps = static_cast<int16_t>(data[4] | (data[5] << 8));
  velocity = static_cast<double>(speed_dps) * M_PI / 180.0;  // Convert DPS to rad/s
  
  // Extract encoder (uint16_t, little-endian)
  uint16_t encoder = static_cast<uint16_t>(data[6] | (data[7] << 8));
  
  // Convert encoder to position (assuming 16-bit encoder, 0-65535)
  // This is a simplified conversion - actual conversion depends on gear ratio
  // For now, we'll use encoder value directly and convert to radians
  // Assuming full range (65535) corresponds to 2*PI radians
  position = static_cast<double>(encoder) * 2.0 * M_PI / 65535.0;
  
  return true;
}

}  // namespace alfa_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)
