# Hardware Interface Refactor Design

**Goal:** 将 `alfa_robot_hardware.cpp` 和 `can_bus.cpp` 拆分重构为低耦合、高内聚的大厂级 ros2_control 硬件插件架构。

**Architecture:** IJoint 虚接口 + RmdDriver/CanopenDriver 独立驱动类 + TrajectoryLogger 独立服务。`AlfaRobotHW` 退化为纯调度者（~100 行），不再感知任何 joint 细节。

**Tech Stack:** C++17, ros2_control SystemInterface, SocketCAN (linux/can.h), rclcpp lifecycle, pluginlib

---

## 1. 问题诊断

### `alfa_robot_hardware.cpp` 现存问题

| 问题 | 位置 |
|------|------|
| 三套并行 buffer（`hw_positions_` / `canopen_positions_` / `hw_states_`） | 全局数据成员 |
| `isCanControlledJoint()` / `isCanopenControlledJoint()` / `isVelocityControlledJoint()` 重复调用 | `on_init`, `export_*`, `read`, `write`, `moveToSafePosition` |
| 硬编码 joint→motor_id 映射（`kRmdMotorIds`） | `on_init` |
| 硬编码减速比 `if (name == "leftarmbase") pos / 3.0` | `read`, `write`, `on_activate` |
| `turn_zero_offset_rad_` 作为顶层特例处理 | `on_activate`, `read`, `write` |
| 轨迹记录服务嵌入 `on_activate` / `on_deactivate` | 违反 SRP |
| 6 张 `std::map` 索引表 | 数据成员 |

### `can_bus.cpp` 现存问题

| 问题 |
|------|
| RMD 协议 + CANopen CiA-402 + 低通滤波 + 轨迹记录全在一个类 |
| 4 个裸 socket FD + availability flag 重复 4 次 |
| `control_mutex_` 保护范围模糊 |
| 调用方必须了解总线拓扑（LEFT/RIGHT/BASE/CANOPEN） |

---

## 2. 目标文件结构

```
alfa_robot_hardware/
├── include/alfa_robot_hardware/
│   ├── joint/
│   │   ├── i_joint.hpp            ← IJoint 纯虚接口
│   │   ├── rmd_joint.hpp          ← RMD 关节
│   │   ├── canopen_joint.hpp      ← CANopen 关节
│   │   └── wheel_joint.hpp        ← 速度/位置透传轮关节
│   ├── driver/
│   │   ├── rmd_driver.hpp         ← RMD SocketCAN 协议
│   │   └── canopen_driver.hpp     ← CANopen CiA-402 协议
│   ├── service/
│   │   └── trajectory_logger.hpp  ← 轨迹记录 + ROS 服务
│   └── alfa_robot_hardware.hpp    ← 精简主类
└── src/
    ├── joint/
    │   ├── rmd_joint.cpp
    │   ├── canopen_joint.cpp
    │   └── wheel_joint.cpp
    ├── driver/
    │   ├── rmd_driver.cpp
    │   └── canopen_driver.cpp
    ├── service/
    │   └── trajectory_logger.cpp
    └── alfa_robot_hardware.cpp    ← ~100 行，纯调度
```

**删除文件：** `src/can_bus.cpp`、`include/alfa_robot_hardware/can_bus.hpp`

---

## 3. 驱动层设计

### `RmdDriver`

```cpp
class RmdDriver {
public:
  struct Config {
    std::string interface;
    uint16_t    max_speed_dps{1800};
    uint8_t     gear_ratio{36};
  };

  explicit RmdDriver(Config cfg);
  ~RmdDriver();

  bool open();
  void close();

  bool enableMotors(const std::vector<uint8_t>& ids);
  void disableMotors(const std::vector<uint8_t>& ids);

  // 批量读：motor_id → position_rad
  std::map<uint8_t, double> readPositions(const std::vector<uint8_t>& ids);

  // 批量写：motor_id → position_rad
  void writePositions(const std::map<uint8_t, double>& cmds);

private:
  Config config_;
  int    socket_fd_{-1};

  bool sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  bool receiveCanFrame(uint32_t& can_id, uint8_t* data, uint8_t& dlc);
  void sendMotorCommand(uint8_t motor_id, uint8_t cmd_byte, const uint8_t* data);
  bool parseMotorAngleReply0x92(const uint8_t* data, double& position_rad);
  void convertPositionToCanFormat0xA4(double position_rad, uint8_t* frame_data);
  void drainResponses(std::map<uint8_t, double>& out);
};
```

**要点：**
- 每个 CAN 总线一个 `RmdDriver` 实例（left/right/base），不再有 `RmdBus` 枚举
- `gear_ratio` 从常量移入 Config，为后续多型号电机留口

### `CanopenDriver`

```cpp
class CanopenDriver {
public:
  struct Config {
    std::string interface;
    uint32_t    profile_velocity{50000};
    uint32_t    profile_accel{50000};
  };

  explicit CanopenDriver(Config cfg);
  ~CanopenDriver();

  bool open();
  void close();

  bool enableNodes(const std::vector<uint8_t>& node_ids);
  void disableNodes(const std::vector<uint8_t>& node_ids);

  // PDO 读（内部执行 SYNC + drain）：node_id → position_m
  std::map<uint8_t, double> readPositions();

  // PDO 写（含 new-setpoint 控制字）：node_id → position_m
  void writePositions(const std::map<uint8_t, double>& cmds_m);

  bool isNodeEnabled(uint8_t node_id) const;

  static constexpr double kPulsesPerMeter = 1000000.0;

private:
  Config                          config_;
  int                             socket_fd_{-1};
  std::set<uint8_t>               enabled_nodes_;
  std::map<uint8_t, CanopenPdoState> pdo_cache_;
  std::map<uint8_t, bool>         new_setpoint_active_;
  std::map<uint8_t, int32_t>      last_target_pulses_;

  // 全部原 canopen* 私有方法迁入
};
```

---

## 4. Joint 抽象层设计

### `IJoint` 纯虚接口

```cpp
class IJoint {
public:
  virtual ~IJoint() = default;

  virtual bool activate() = 0;
  virtual void deactivate() = 0;

  virtual void read(double dt) = 0;
  virtual void write(double dt) = 0;

  virtual std::vector<hardware_interface::StateInterface>   exportStateInterfaces() = 0;
  virtual std::vector<hardware_interface::CommandInterface> exportCommandInterfaces() = 0;

  virtual bool moveToSafePosition(double target_rad, double timeout_s) = 0;

  const std::string& name() const { return name_; }

protected:
  explicit IJoint(std::string name) : name_(std::move(name)) {}
  std::string name_;
};
```

### `RmdJoint`

```cpp
class RmdJoint final : public IJoint {
public:
  struct Config {
    uint8_t motor_id;
    double  gear_ratio{1.0};
    double  zero_offset_rad{0.0};   // 替代 turn_zero_offset_rad_ 特例
    double  filter_cutoff_hz{0.0};  // 0 = 不滤波
  };

  RmdJoint(std::string name, Config cfg, RmdDriver& driver);
  // 实现 IJoint 全部接口
  // read(): 读位置 → 应用 zero_offset → 计算速度/加速度 → 可选低通滤波
  // write(): 应用 zero_offset → 可选低通滤波 → 发指令

private:
  Config     config_;
  RmdDriver& driver_;   // 非 owner，引用由 AlfaRobotHW 管理

  double position_{0.0}, velocity_{0.0}, acceleration_{0.0};
  double position_cmd_{0.0};
  double prev_position_{0.0}, prev_velocity_{0.0}, prev_filtered_{0.0};
  bool   filter_initialized_{false}, first_read_{true};
};
```

### `CanopenJoint`

```cpp
class CanopenJoint final : public IJoint {
public:
  struct Config {
    uint8_t node_id;
    double  gear_ratio{1.0};        // leftarmbase/rightarmbase = 3.0（配置化）
    double  filter_cutoff_hz{0.0};
  };

  CanopenJoint(std::string name, Config cfg, CanopenDriver& driver);
  // read(): 从 driver PDO cache 取值 → 应用 gear_ratio → 计算速度/加速度
  // write(): 应用 gear_ratio → 可选滤波 → 发指令

private:
  Config          config_;
  CanopenDriver&  driver_;

  double position_{0.0}, velocity_{0.0}, acceleration_{0.0};
  double position_cmd_{0.0};
  double prev_position_{0.0}, prev_velocity_{0.0}, prev_filtered_{0.0};
  bool   filter_initialized_{false}, first_read_{true};
};
```

### `WheelJoint`（原 legacy joints）

```cpp
class WheelJoint final : public IJoint {
public:
  enum class ControlMode { Velocity, Position };

  WheelJoint(std::string name, ControlMode mode);
  // read(dt): Velocity 模式积分位置；Position 模式直通
  // write(): 空实现（无实际硬件）
  // moveToSafePosition(): 立即返回 true

private:
  ControlMode mode_;
  double position_{0.0}, velocity_{0.0}, acceleration_{0.0};
  double position_cmd_{0.0}, velocity_cmd_{0.0};
  double prev_position_{0.0}, prev_velocity_{0.0};
};
```

---

## 5. 服务层设计

### `TrajectoryLogger`

```cpp
class TrajectoryLogger {
public:
  explicit TrajectoryLogger(rclcpp::Node::SharedPtr node);
  ~TrajectoryLogger();

  // on_activate 时注入回调，解耦 Logger 与 Driver
  void attachCallbacks(std::function<void(uint8_t)> on_start,
                       std::function<void()>        on_stop);

  // write() 循环里按需调用
  void record(uint8_t motor_id, double time_s, double p_raw, double p_cmd);

private:
  rclcpp::Node::SharedPtr                             node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  dump_srv_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread                                         thread_;

  std::atomic<bool>              active_{false};
  uint8_t                        motor_id_{0};
  double                         time_s_{0.0};
  std::vector<TrajectoryLogEntry> log_;
  mutable std::mutex             log_mutex_;

  void dumpToFile(const std::string& path) const;
};
```

---

## 6. `AlfaRobotHW` 精简版

```cpp
class AlfaRobotHW : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  // 驱动（生命周期 owner）
  std::unique_ptr<RmdDriver>      rmd_left_, rmd_right_, rmd_base_;
  std::unique_ptr<CanopenDriver>  canopen_;

  // Joint 列表（唯一数据结构）
  std::vector<std::unique_ptr<IJoint>> joints_;

  // 服务
  std::unique_ptr<TrajectoryLogger> traj_logger_;

  // 安全停机配置
  std::map<std::string, double> safe_positions_;
  bool use_safe_shutdown_{false};

  void buildJoints(const hardware_interface::HardwareInfo& info);
};
```

**`read` / `write` 最终形态：**

```cpp
return_type AlfaRobotHW::read(const rclcpp::Time&, const rclcpp::Duration& period) {
  double dt = period.nanoseconds() > 0 ? period.seconds() : 0.0;
  for (auto& joint : joints_) joint->read(dt);
  return return_type::OK;
}

return_type AlfaRobotHW::write(const rclcpp::Time&, const rclcpp::Duration& period) {
  double dt = period.nanoseconds() > 0 ? period.seconds() : 0.005;
  for (auto& joint : joints_) joint->write(dt);
  return return_type::OK;
}
```

---

## 7. 依赖关系图

```
AlfaRobotHW (owner)
 ├── RmdDriver: left / right / base
 ├── CanopenDriver
 ├── joints_: vector<unique_ptr<IJoint>>
 │    ├── RmdJoint(turn)          → RmdDriver& (base)
 │    ├── RmdJoint(leftjoint2-4)  → RmdDriver& (left)
 │    ├── RmdJoint(rightjoint2-4) → RmdDriver& (right)
 │    ├── CanopenJoint(updown)         → CanopenDriver&
 │    ├── CanopenJoint(leftarmbase)    → CanopenDriver& (gear=3.0)
 │    ├── CanopenJoint(leftjoint1)     → CanopenDriver&
 │    ├── CanopenJoint(rightarmbase)   → CanopenDriver& (gear=3.0)
 │    ├── CanopenJoint(rightjoint1)    → CanopenDriver&
 │    ├── WheelJoint(left_back)        Velocity
 │    ├── WheelJoint(left_forward)     Velocity
 │    ├── WheelJoint(right_back)       Velocity
 │    └── WheelJoint(right_forward)    Velocity
 └── TrajectoryLogger (独立，仅 on_activate 注入回调)
```

所有依赖单向向下，无循环依赖。

---

## 8. 删除清单

| 删除 | 替代 |
|------|------|
| `can_bus.hpp` / `can_bus.cpp` | `rmd_driver` + `canopen_driver` |
| `isCanControlledJoint()` | joint 类型本身 |
| `isCanopenControlledJoint()` | joint 类型本身 |
| `isVelocityControlledJoint()` | `WheelJoint::ControlMode` |
| `hw_positions_` / `canopen_positions_` / `hw_states_` | 各 joint 自��� |
| `joint_to_motor_id_` / `joint_to_state_index_` 等 6 张 map | `joints_` 列表 |
| `turn_zero_offset_rad_` 特例 | `RmdJoint::Config::zero_offset_rad` |
| `if (name == "leftarmbase") pos / 3.0` 内联逻辑 | `CanopenJoint::Config::gear_ratio` |
| `traj_log_node_` / `traj_log_executor_` / `traj_log_thread_` | `TrajectoryLogger` |

---

## 9. CMakeLists 变更

```cmake
add_library(alfa_robot_hardware SHARED
  src/alfa_robot_hardware.cpp
  src/joint/rmd_joint.cpp
  src/joint/canopen_joint.cpp
  src/joint/wheel_joint.cpp
  src/driver/rmd_driver.cpp
  src/driver/canopen_driver.cpp
  src/service/trajectory_logger.cpp
)
```

---

## 10. 测试策略

- `RmdDriver` / `CanopenDriver`：单元测试 mock SocketCAN（`socket()` 打桩）
- `RmdJoint` / `CanopenJoint`：注入 mock Driver，测试 read/write/gear_ratio/zero_offset 逻辑
- `WheelJoint`：纯逻辑测试，无外部依赖
- `TrajectoryLogger`：测试 record → dump 文件内容
- `AlfaRobotHW`：沿用 `ros2_control_test_assets` URDF 加载测试
