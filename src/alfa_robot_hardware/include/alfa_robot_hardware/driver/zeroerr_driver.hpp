// Copyright (c) 2026, alfa
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef ALFA_ROBOT_HARDWARE__DRIVER__ZEROERR_DRIVER_HPP_
#define ALFA_ROBOT_HARDWARE__DRIVER__ZEROERR_DRIVER_HPP_

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace alfa_robot_hardware
{

/**
 * @brief ZeroErr (零差云控) 自定义 CAN 协议驱动
 *
 * CAN 通信参数:
 *   - 波特率: 1 Mbps
 *   - 帧类型: 标准帧 (11-bit ID)
 *   - 命令 ID: 0x640 + Node ID
 *   - 响应 ID: 0x5C0 + Node ID
 *
 * 关键参数:
 *   - 编码器分辨率: 524,288 count/rev
 *   - 减速比: 200:1 (可配置)
 *   - 输出轴分辨率: 524,288 * 200 = 104,857,600 count/rev
 */
class ZeroerrDriver
{
public:
  struct Config {
    std::string interface;           // CAN 接口名 (e.g., "can0")
    uint32_t gear_ratio{200};        // 减速比 (默认 200:1)
    uint32_t encoder_resolution{524288};  // 编码器分辨率 (脉冲/圈)
  };

  // 输出轴每转一圈的脉冲数 = encoder_resolution * gear_ratio
  // 默认: 524288 * 200 = 104,857,600
  // 每弧度对应的脉冲数: 104857600 / (2 * PI) ≈ 16,699,281

  explicit ZeroerrDriver(Config cfg);
  ~ZeroerrDriver();

  ZeroerrDriver(const ZeroerrDriver &) = delete;
  ZeroerrDriver & operator=(const ZeroerrDriver &) = delete;

  /// 打开 CAN 接口
  bool open();

  /// 关闭 CAN 接口
  void close();

  /// 使能电机 (发送 0x01 命令)
  bool enableMotors(const std::vector<uint8_t> & node_ids);

  /// 失能电机 (发送 0x01 命令, 数据 00)
  void disableMotors(const std::vector<uint8_t> & node_ids);

  /// 切换到位置模式 (发送 0x4E 命令)
  bool setPositionMode(const std::vector<uint8_t> & node_ids);

  /// 读取实际位置 (发送 0x02 命令)
  /// 返回 node_id -> position_count (脉冲数)
  std::map<uint8_t, int32_t> readPositions(const std::vector<uint8_t> & node_ids);

  /// 从缓存获取位置 (无 CAN I/O)
  bool getCachedPosition(uint8_t node_id, int32_t & position_count) const;

  /// 写入目标位置并开始运动
  /// 1. 发送 0x86 设定绝对位置
  /// 2. 发送 0x83 开始运动
  void writePositions(const std::map<uint8_t, int32_t> & position_cmds);

  /// 停止运动 (发送 0x84 命令)
  void stopMotors(const std::vector<uint8_t> & node_ids);

  /// 读取错误码 (发送 0x1F 命令)
  uint16_t readErrorCode(uint8_t node_id);

  /// 读取运行状态 (发送 0x20 命令)
  /// 返回: 0=停止, 1=运动中, 3=重复停止中
  uint8_t readRunStatus(uint8_t node_id);

  /// 检查接口是否打开
  bool isOpen() const { return socket_fd_ >= 0; }

  /// 检查节点是否已使能
  bool isNodeEnabled(uint8_t node_id) const;

  /// 获取输出轴每弧度对应的脉冲数
  double getCountsPerRadian() const { return counts_per_radian_; }

private:
  Config config_;
  int socket_fd_{-1};
  std::set<uint8_t> enabled_nodes_;
  std::map<uint8_t, int32_t> position_cache_;
  double counts_per_radian_;  // 每弧度对应的脉冲数

  /// 发送 CAN 帧
  bool sendCanFrame(uint32_t can_id, const uint8_t * data, uint8_t dlc);

  /// 接收 CAN 帧
  bool receiveCanFrame(uint32_t & can_id, uint8_t * data, uint8_t & dlc);

  /// 发送命令并等待响应
  bool sendCommandAndWait(uint8_t node_id, const uint8_t * send_data, uint8_t send_dlc,
                          uint8_t * resp_data, uint8_t & resp_dlc, int timeout_ms = 100);

  /// 发送双字节命令 (e.g., 0x00 0x02 for read position)
  bool sendShortCommand(uint8_t node_id, uint8_t cmd_high, uint8_t cmd_low);

  /// 发送带数据的命令 (数据长度 1-7 字节)
  bool sendDataCommand(uint8_t node_id, uint8_t cmd_byte, const uint8_t * data, uint8_t data_len);

  /// 发送 6 字节命令并等待响应
  bool send6ByteCommand(uint8_t node_id, const uint8_t data[6]);

  /// 解析位置响应 (格式: [D3 D2 D1 D0 3E])
  bool parsePositionResponse(const uint8_t * data, uint8_t dlc, int32_t & position);

  /// 脉冲数转弧度
  double countsToRadians(int32_t counts) const;

  /// 弧度转脉冲数
  int32_t radiansToCounts(double radians) const;
};

}  // namespace alfa_robot_hardware

#endif  // ALFA_ROBOT_HARDWARE__DRIVER__ZEROERR_DRIVER_HPP_
