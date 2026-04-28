// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__DRIVER__CYLINDER_DRIVER_HPP_
#define ALFA_ROBOT_HARDWARE__DRIVER__CYLINDER_DRIVER_HPP_

#include <cstdint>
#include <string>

namespace alfa_robot_hardware
{

/**
 * @brief IDS830ABS 电缸驱动器（自定义 CAN 协议）
 *
 * 通信参数:
 *   - 波特率：1 Mbps
 *   - 发送 ID: Node ID (e.g., 0x03)
 *   - 响应 ID: Node ID + 0x100 (e.g., 0x103)
 *   - 功能码：0x1A(写), 0x2A(读), 0x1B(写响应), 0x2B(读响应)
 *
 * 关键寄存器:
 *   - 0x00: 控制使能 (1=使能，0=失能)
 *   - 0x10: 目标速度
 *   - 0x50: 目标位置高 16 位
 *   - 0x05: 目标位置低 16 位
 *   - 0xE8: 实际位置高 16 位 (反馈)
 *   - 0xE9: 实际位置低 16 位 (反馈)
 *
 * 位置单位:
 *   - 200,000 脉冲/m (10000 脉冲/50mm)
 */
class CylinderDriver
{
public:
  struct Config {
    std::string interface;    // CAN 接口名 (e.g., "can0")
    uint8_t node_id{3};       // 从站 Node ID (默认 3)
    double pulses_per_meter{2000000.0};  // 脉冲/米 (10000脉冲/5mm)
    int32_t zero_offset{0};   // 零点偏置 (脉冲数)，默认 0
  };

  explicit CylinderDriver(Config cfg);
  ~CylinderDriver();

  CylinderDriver(const CylinderDriver &) = delete;
  CylinderDriver & operator=(const CylinderDriver &) = delete;

  /// 打开 CAN 接口
  bool open();

  /// 关闭 CAN 接口
  void close();

  /// 使能电缸 (写寄存器 0x00 = 1)
  bool enable();

  /// 失能电缸 (写寄存器 0x00 = 0)
  void disable();

  /// 读取实际位置 (读寄存器 E8+E9, 返回米)
  bool readPosition(double & position_m);

  /// 写入目标位置 (写寄存器 50+05, 单位米)
  bool writePosition(double target_m);

  /// 设置目标速度 (写寄存器 0x10, 单位 m/s)
  bool setVelocity(double velocity_m);

  /// 检查接口是否打开
  bool isOpen() const { return socket_fd_ >= 0; }

  /// 检查是否已使能
  bool isEnabled() const { return enabled_; }

  /// 获取缓存位置 (无 CAN I/O)
  bool getCachedPosition(double & position_m) const;

private:
  Config config_;
  int socket_fd_{-1};
  bool enabled_{false};
  double position_cache_m_{0.0};  // 位置缓存 (米，已减去零点偏置)

  /// 发送 CAN 帧
  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

  /// 接收 CAN 帧 (带超时)
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc, int timeout_ms);

  /// 写单个寄存器
  bool writeRegister(uint8_t reg_addr, int16_t value);

  /// 写两个寄存器 (一次事务)
  bool writeTwoRegisters(uint8_t reg1_addr, int16_t value1, uint8_t reg2_addr, int16_t value2);

  /// 读两个寄存器
  bool readTwoRegisters(uint8_t reg1_addr, int16_t & value1, uint8_t reg2_addr, int16_t & value2);

  /// 脉冲转米 (已减去零点偏置)
  double pulsesToMeters(int32_t pulses) const;

  /// 米转脉冲 (已加上零点偏置)
  int32_t metersToPulses(double meters) const;

  /// 读取原始脉冲数 (未偏置)
  bool readRawPulses(int32_t & pulses);
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__DRIVER__CYLINDER_DRIVER_HPP_
