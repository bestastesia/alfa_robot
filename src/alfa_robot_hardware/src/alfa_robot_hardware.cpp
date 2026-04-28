#include "alfa_robot_hardware/alfa_robot_hardware.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alfa_robot_hardware
{

hardware_interface::CallbackReturn AlfaRobotHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Defaults
  rmd_left_cfg_        = {"can0", 1800};
  rmd_right_cfg_       = {"can1", 1800};
  rmd_base_cfg_        = {"can2", 1800};
  canopen_cfg_         = {"can3", 50000, 50000};
  canopen_plate_cfg_   = {"can4", 50000, 50000};
  zeroerr_left_cfg_    = {"can0", 200, 524288};  // ZeroErr on can0: gear_ratio=200, encoder=524288
  cylinder_cfg_        = {"can0", 3, 2000000.0, -24995000};   // Cylinder on can0: Node 3, 10000 pulses/5mm = 2M pulses/m, zero_offset=-24900000

  for (const auto & [key, val] : info_.hardware_parameters) {
    if      (key == "can_interface_left")    { rmd_left_cfg_.interface       = val; }
    else if (key == "can_interface_right")   { rmd_right_cfg_.interface      = val; }
    else if (key == "can_interface_base")    { rmd_base_cfg_.interface       = val; }
    else if (key == "can_interface_canopen") { canopen_cfg_.interface        = val; }
    else if (key == "can_interface_plate")   { canopen_plate_cfg_.interface  = val; }
    else if (key == "max_speed_dps") {
      try {
        uint16_t v = static_cast<uint16_t>(std::stoul(val));
        rmd_left_cfg_.max_speed_dps = rmd_right_cfg_.max_speed_dps =
          rmd_base_cfg_.max_speed_dps = v;
      } catch (...) {}
    }
    else if (key == "canopen_profile_velocity") {
      try { canopen_cfg_.profile_velocity = static_cast<uint32_t>(std::stoul(val)); }
      catch (...) {}
    }
    else if (key == "canopen_profile_accel") {
      try { canopen_cfg_.profile_accel = static_cast<uint32_t>(std::stoul(val)); }
      catch (...) {}
    }
    else if (key == "use_safe_shutdown") {
      use_safe_shutdown_ = (val == "true");
    }
    else if (key.find("safe_position_") == 0) {
      try { safe_positions_[key.substr(14)] = std::stod(val); }
      catch (...) {}
    }
  }

  // Create drivers and joints here so export_state/command_interfaces() works
  // immediately after on_init (ros2_control calls them before on_configure).
  rmd_left_       = std::make_unique<RmdDriver>(rmd_left_cfg_);
  rmd_right_      = std::make_unique<RmdDriver>(rmd_right_cfg_);
  rmd_base_       = std::make_unique<RmdDriver>(rmd_base_cfg_);
  canopen_        = std::make_unique<CanopenDriver>(canopen_cfg_);
  canopen_plate_  = std::make_unique<CanopenDriver>(canopen_plate_cfg_);
  // Mixed protocol on can0: ZeroErr driver for Node 1,2 (custom CAN protocol)
  zeroerr_left_   = std::make_unique<ZeroerrDriver>(zeroerr_left_cfg_);
  // Cylinder on can0: Node 3 (IDS830ABS linear actuator)
  cylinder_       = std::make_unique<CylinderDriver>(cylinder_cfg_);
  buildJoints();

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "on_init OK");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_configure(
  const rclcpp_lifecycle::State &)
{
  rmd_left_->open();    // Non-fatal if bus absent
  rmd_right_->open();
  rmd_base_->open();
  canopen_->open();
  canopen_plate_->open();
  zeroerr_left_->open();  // ZeroErr motors on can0
  cylinder_->open();      // Cylinder on can0 (Node 3)

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"),
    "on_configure OK, %zu joints", joints_.size());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Enable motors per bus
  // Mixed protocol on can0: Node 1,2 are ZeroErr (custom CAN), Node 3 is Cylinder (IDS830ABS), Node 4 is RMD (leftjoint5)
  cylinder_->enable();                   // Node 3 (leftjoint4 - cylinder)
  cylinder_->setVelocity(0.05);          // Set velocity to 0.05 m/s (50 mm/s) for cylinder
  rmd_left_->enableMotors({4});          // Node 4 (leftjoint5 - RMD protocol)

  // ZeroErr motors on can0: Full initialization sequence per datasheet
  zeroerr_left_->enableMotors({1, 2});     // Node 1,2 (leftjoint2/3) - 01 00 00 00 00 01
  zeroerr_left_->setPositionMode({1, 2});  // 00 4E 00 00 00 03
  zeroerr_left_->setMotionMode({1, 2}, 1); // 00 8D 00 00 00 01 (1=absolute position)
  zeroerr_left_->setMotionParams({1, 2});  // 00 88/89/8A - accel/decel/velocity

  rmd_right_->enableMotors({4, 5, 6});   // rightjoint2/3/4
  rmd_base_->enableMotors({1});           // turn
  canopen_->enableNodes({1, 2, 3, 4, 5});
  canopen_plate_->enableNodes({1});  // plate

  // Activate all joints (reads initial position)
  for (auto & joint : joints_) { joint->activate(); }

  // Capture "turn" joint current position as software zero
  for (auto & joint : joints_) {
    if (joint->name() == "turn") {
      auto * rmd_joint = dynamic_cast<RmdJoint *>(joint.get());
      if (rmd_joint) { rmd_joint->captureCurrentPositionAsZero(); }
    }
  }


  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlfaRobotHW::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (use_safe_shutdown_ && !safe_positions_.empty()) {
    moveAllToSafePositions(5.0);
  }

  // Mixed protocol on can0
  cylinder_->disable();                  // Node 3 (leftjoint4 - cylinder)
  rmd_left_->disableMotors({4});         // Node 4 (leftjoint5 - RMD)
  zeroerr_left_->disableMotors({1, 2});  // Node 1,2 (leftjoint2/3)

  rmd_right_->disableMotors({4, 5, 6});
  rmd_base_->disableMotors({1});
  canopen_->disableNodes({1, 2, 3, 4, 5});
  canopen_plate_->disableNodes({1});  // plate

  rmd_left_->close();
  rmd_right_->close();
  rmd_base_->close();
  canopen_->close();
  canopen_plate_->close();
  zeroerr_left_->close();
  cylinder_->close();

  RCLCPP_INFO(rclcpp::get_logger("AlfaRobotHW"), "Hardware deactivated");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlfaRobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (auto & joint : joints_) {
    auto joint_si = joint->exportStateInterfaces();
    for (auto & iface : joint_si) { si.push_back(std::move(iface)); }
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> AlfaRobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & joint : joints_) {
    auto joint_ci = joint->exportCommandInterfaces();
    for (auto & iface : joint_ci) { ci.push_back(std::move(iface)); }
  }
  return ci;
}

hardware_interface::return_type AlfaRobotHW::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.0;
  // One batched read per bus - sends all 0x92, then drains with a poll() budget.
  // Joints subsequently read from the driver's position cache.
  // Mixed protocol on can0: Node 1,2 via ZeroErr (custom CAN), Node 3 via Cylinder (IDS830ABS), Node 4 via RMD (leftjoint5)
  // IMPORTANT: All can0 reads must be sequential to avoid CAN bus contention
  rmd_left_->readPositions({4});         // can0: Node 4 (leftjoint5 - RMD)
  {                                      // can0: Node 3 (leftjoint4 - Cylinder)
    double dummy;
    cylinder_->readPosition(dummy);
  }
  zeroerr_left_->readPositions({1, 2});  // can0: ZeroErr motors (Node 1,2)

  rmd_right_->readPositions({4, 5, 6});
  rmd_base_->readPositions({1});
  canopen_->readPositions();             // can3: one SYNC per cycle, updates PDO cache
  canopen_plate_->readPositions();       // can4: plate bus

  for (auto & joint : joints_) { joint->read(dt); }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlfaRobotHW::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = (period.nanoseconds() > 0) ? period.seconds() : 0.005;
  for (auto & joint : joints_) { joint->write(dt); }
  return hardware_interface::return_type::OK;
}

// ── Private ──────────────────────────────────────────────────────────────────

void AlfaRobotHW::buildJoints()
{
  // RMD joints - base bus (can2)
  joints_.push_back(std::make_unique<RmdJoint>("turn",
    RmdJoint::Config{1, 0.0, 0.0, -1.0, 2.394}, *rmd_base_));

  // Left bus (can0) - mixed protocol
  // Node 1,2: ZeroErr rotary motors (gear_ratio=200:1, encoder_resolution=524288 pulses/rev)
  // 零点位置：通过 CAN 命令手动读取 (cansend can0 64X#00.02)
  // Node 1 (leftjoint2): 0x00040000 = 262,144 脉冲 (机械零点)
  // Node 2 (leftjoint3): 0x00040000 = 262,144 脉冲 (机械零点)
  joints_.push_back(std::make_unique<ZeroerrJoint>("leftjoint2",
    ZeroerrJoint::Config{1, 0.0, -1.0, 262144}, *zeroerr_left_));
  joints_.push_back(std::make_unique<ZeroerrJoint>("leftjoint3",
    ZeroerrJoint::Config{2, 0.0, -1.0, 262144}, *zeroerr_left_));
  // Node 3: IDS830ABS Cylinder (leftjoint4 - linear actuator, 15cm travel)
  joints_.push_back(std::make_unique<CylinderJoint>("leftjoint4",
    CylinderJoint::Config{3, 0.0, 1.0, 0.0, 0.15}, *cylinder_));
  // Node 4: RMD motor (leftjoint5 - rotary)
  joints_.push_back(std::make_unique<RmdJoint>("leftjoint5",
    RmdJoint::Config{4, 0.0, 0.0, -1.0, 0.0}, *rmd_left_));

  // RMD joints - right bus (can1)
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint2",
    RmdJoint::Config{4, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint3",
    RmdJoint::Config{5, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));
  joints_.push_back(std::make_unique<RmdJoint>("rightjoint4",
    RmdJoint::Config{6, 0.0, 0.0, -1.0, 0.0}, *rmd_right_));

  // CANopen joints (can3) - linear actuators
  joints_.push_back(std::make_unique<CanopenJoint>("updown",
    CanopenJoint::Config{1, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("leftarmbase",
    CanopenJoint::Config{2, 3.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("leftjoint1",
    CanopenJoint::Config{3, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("rightarmbase",
    CanopenJoint::Config{4, 3.0, 0.0, 0.0, -1.0}, *canopen_));
  joints_.push_back(std::make_unique<CanopenJoint>("rightjoint1",
    CanopenJoint::Config{5, 1.0, 0.0, 0.0, -1.0}, *canopen_));
  // plate - separate CANopen bus (can4), node 1
  joints_.push_back(std::make_unique<CanopenJoint>("plate",
    CanopenJoint::Config{1, 1.0, 0.0, 0.0, -1.0}, *canopen_plate_));

}

bool AlfaRobotHW::moveAllToSafePositions(double timeout_s)
{
  bool all_ok = true;
  for (const auto & [joint_name, safe_pos] : safe_positions_) {
    for (auto & joint : joints_) {
      if (joint->name() == joint_name) {
        if (!joint->moveToSafePosition(safe_pos, timeout_s)) { all_ok = false; }
        break;
      }
    }
  }
  return all_ok;
}

}  // namespace alfa_robot_hardware

PLUGINLIB_EXPORT_CLASS(alfa_robot_hardware::AlfaRobotHW, hardware_interface::SystemInterface)