# CANopen PDO 动态映射重构技术文档

## 概述

本次重构将 `alfa_robot_hardware` 的 CANopen 通信从阻塞式 SDO 切换为实时 PDO，并采用**严格动态映射**策略。

---

## 架构决策

### 1. 动态映射 vs 默认映射

**选择：严格动态映射**

- **优势**：
  - 完全掌控 PDO 字典，不依赖固件默认配置
  - 显式配置 Inhibit Time 防止总线过载
  - 失败时立即发现问题，不会在运行时产生隐蔽错误

- **实现**：在 `on_activate()` Phase 2 中，通过 13 步 SDO 操作完成 TxPDO1 + RxPDO1 映射

### 2. SYNC 触发模式

**选择：SYNC 同步触发（Transmission Type = 1）**

- **TxPDO1**：每个 SYNC 触发一次位置反馈
- **RxPDO1**：接收后在下一个 SYNC 生效

**理由**：
- 5 个关节同时采样/执行，保证机械同步性
- 避免 CAN 总线时序差异导致的关节抖动
- 符合 CiA 402 多轴同步控制最佳实践

### 3. Inhibit Time 配置

**设置：1.0ms（0x1800:03 = 10 × 100μs）**

防止 TxPDO 过载总线。即使 SYNC 频率高于 1kHz 或事件触发失控，单个节点的 TxPDO 发送间隔不会低于 1ms。

---

## PDO 映射字典

### TxPDO1 (从站 → 主站)

| 对象 | 索引 | 子索引 | 类型 | 位宽 | 映射描述符 |
|------|------|--------|------|------|-----------|
| 状态字 | 0x6041 | 0x00 | UINT16 | 16 | 0x60410010 |
| 实际位置 | 0x6064 | 0x00 | INT32 | 32 | 0x60640020 |

**通信参数 (0x1800)**：
- Sub 1 (COB-ID): `0x180 + NodeID`
- Sub 2 (Transmission Type): `1` (SYNC)
- Sub 3 (Inhibit Time): `10` (1.0ms)

**帧格式**：
```
COB-ID: 0x180 + NodeID
DLC: 6 bytes
Data: [SW_lo, SW_hi, Pos_b0, Pos_b1, Pos_b2, Pos_b3]
```

### RxPDO1 (主站 → 从站)

| 对象 | 索引 | 子索引 | 类型 | 位宽 | 映射描述符 |
|------|------|--------|------|------|-----------|
| 控制字 | 0x6040 | 0x00 | UINT16 | 16 | 0x60400010 |
| 目标位置 | 0x607A | 0x00 | INT32 | 32 | 0x607A0020 |

**通信参数 (0x1400)**：
- Sub 1 (COB-ID): `0x200 + NodeID`
- Sub 2 (Transmission Type): `1` (SYNC)

**帧格式**：
```
COB-ID: 0x200 + NodeID
DLC: 6 bytes
Data: [CW_lo, CW_hi, Target_b0, Target_b1, Target_b2, Target_b3]
```

---

## 动态映射配置流程

### `canopenConfigurePdo()` 13 步序列

#### TxPDO1 配置 (Steps 1-7)

1. **失能 TxPDO1**：`0x1800:01 = (0x180+NodeID) | 0x80000000`
2. **设置传输类型**：`0x1800:02 = 1` (SYNC)
3. **设置 Inhibit Time**：`0x1800:03 = 10` (1.0ms)
4. **清空映射**：`0x1A00:00 = 0`
5. **映射状态字**：`0x1A00:01 = 0x60410010`
6. **映射实际位置**：`0x1A00:02 = 0x60640020`
7. **激活映射**：`0x1A00:00 = 2`，然后 `0x1800:01 = 0x180+NodeID`

#### RxPDO1 配置 (Steps 8-13)

8. **失能 RxPDO1**：`0x1400:01 = (0x200+NodeID) | 0x80000000`
9. **设置传输类型**：`0x1400:02 = 1` (SYNC)
10. **清空映射**：`0x1600:00 = 0`
11. **映射控制字**：`0x1600:01 = 0x60400010`
12. **映射目标位置**：`0x1600:02 = 0x607A0020`
13. **激活映射**：`0x1600:00 = 2`，然后 `0x1400:01 = 0x200+NodeID`

**错误处理**：任何步骤失败立即返回 `false`，节点被排除在 `canopen_enabled_nodes_` 之外。

---

## 实时 PDO 通信

### `write()` — 发送 RxPDO1

```cpp
// 位操作打包（显式，无 memcpy）
uint8_t data[8] = {0};
data[0] = controlword & 0xFFu;           // CW 低字节
data[1] = (controlword >> 8) & 0xFFu;    // CW 高字节
uint32_t pos_u = static_cast<uint32_t>(target_pulses);
data[2] = pos_u & 0xFFu;                 // 位置 LSB
data[3] = (pos_u >> 8) & 0xFFu;
data[4] = (pos_u >> 16) & 0xFFu;
data[5] = (pos_u >> 24) & 0xFFu;         // 位置 MSB
sendCanFrame(socket_fd, 0x200 + node_id, data, 6);
```

**控制字逻辑**（PP 模式 bit4 上升沿）：
- 目标位置变化时：先发 `0x002F`（bit4=0），下一周期发 `0x003F`（bit4=1）产生上升沿
- 目标不变时：持续发 `0x003F`（bit4=1），不产生新上升沿，电机继续当前运动

### `read()` — 接收 TxPDO1

```cpp
// 1. 预清空旧帧
canopenPdoReceiveAll(socket_fd);

// 2. 广播 SYNC（同时让 RxPDO 生效 + 触发 TxPDO）
canopenSendSync(socket_fd);

// 3. 等待 1.5ms（5 节点 × 230μs/帧 ≈ 1.2ms）
usleep(1500);

// 4. 非阻塞接收所有 TxPDO1
canopenPdoReceiveAll(socket_fd);

// 5. 从缓存更新状态向量
for (auto & pair : canopen_joint_to_node_id_) {
    auto & pdo = canopen_pdo_state_[pair.second];
    if (pdo.valid) {
        canopen_positions_[idx] = pdo.actual_position_pulses / kCanopenPulsesPerMeter;
    }
}
```

**位操作解包**（显式，无 memcpy）：
```cpp
uint16_t statusword = raw[0] | (raw[1] << 8);
uint32_t pos_u = raw[2] | (raw[3] << 8) | (raw[4] << 16) | (raw[5] << 24);
int32_t actual_position = static_cast<int32_t>(pos_u);
```

---

## 性能对比

| 指标 | SDO (旧) | PDO (新) |
|------|----------|----------|
| `read()` 耗时 | ~10ms (5×SDO读) | ~1.7ms (SYNC+drain) |
| `write()` 耗时 | ~15ms (5×3×SDO写) | ~0.8ms (5×RxPDO) |
| 总 CANopen 开销 | ~25ms | ~2.5ms |
| 100Hz 控制周期 | **不可行** | **可行** |
| 阻塞风险 | 高（SDO超时卡死） | 无（非阻塞drain） |

---

## 关键代码位置

| 功能 | 文件 | 行号 |
|------|------|------|
| 动态映射配置 | `alfa_robot_hardware.cpp` | 1341-1550 |
| `on_activate()` Phase 2 | `alfa_robot_hardware.cpp` | 481-496 |
| RxPDO1 发送（位操作） | `alfa_robot_hardware.cpp` | 1559-1585 |
| TxPDO1 接收（位操作） | `alfa_robot_hardware.cpp` | 1587-1635 |
| `read()` SYNC 逻辑 | `alfa_robot_hardware.cpp` | 741-785 |
| `write()` 控制字逻辑 | `alfa_robot_hardware.cpp` | 878-914 |

---

## 故障排查

### 问题 1：节点在 Phase 2 被排除

**症状**：日志显示 `PDO dynamic mapping FAILED for node X`

**排查**：
1. 检查节点是否在 Pre-Operational 状态（NMT reset 后 150ms 内）
2. 使用 `candump can3` 监控 SDO 响应（0x580+NodeID）
3. 检查节点是否支持动态 PDO 映射（部分固件锁定）

### 问题 2：TxPDO 未收到

**症状**：`canopen_pdo_state_[node_id].valid` 始终为 `false`

**排查**：
1. 确认 SYNC 正常发送（`candump can3 | grep 080`）
2. 检查 TxPDO COB-ID 是否正确（`0x180+NodeID`）
3. 验证 Transmission Type = 1（`cansend can3 600#40001801` 读取 0x1800:02）

### 问题 3：位置抖动

**症状**：Rviz2 中机器人模型抖动

**排查**：
1. 检查 `read()` 中是否正确预清空旧帧（line 748）
2. 验证 `write()` 中 bit4 上升沿逻辑（line 896-912）
3. 确认 `usleep(1500)` 足够长（500kbps CAN 总线）

---

## 测试验证

### 单元测试

```bash
# 1. 验证 PDO 映射配置
ros2 run alfa_robot_hardware test_pdo_config

# 2. 监控 CAN 总线流量
candump can3 -L

# 3. 检查 SYNC 频率
candump can3 | grep 080 | pv -l -i 1
```

### 集成测试

```bash
# 启动硬件接口
ros2 launch alfa_robot_bringup hardware.launch.py

# 检查日志中的 PDO 配置成功信息
# 预期输出：
# [INFO] Node 1: PDO dynamic mapping complete — TxPDO1[0x181]={statusword,position} ...
# [INFO] CANopen motors: 5/5 enabled with PDO
```

---

## 参考文档

- CiA 301 v4.2.0 §7.2.15 (PDO Mapping)
- CiA 402 v4.1.0 §6.3.1 (Profile Position Mode)
- 雷赛 CANopen 手册 (Leisai Servo Drive CANopen Manual)
- `alfa_robot_hardware.hpp` (类定义)

---

**最后更新**：2026-03-05
**作者**：ROS2 Hardware Interface Team
