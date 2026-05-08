# IK 求解服务

## 源码位置（对接用）

| 文件 | 路径 |
|------|------|
| IK 求解器类 | [ik_solver.h](../../scripts/ik_benchmark/include/ik_benchmark/ik_solver.h) / [ik_solver.cpp](../../scripts/ik_benchmark/src/ik_solver.cpp) |
| Demo 可执行 | [demo_single.cpp](../../scripts/ik_benchmark/src/demo_single.cpp) |
| Benchmark 可执行 | [benchmark_main.cpp](../../scripts/ik_benchmark/src/benchmark_main.cpp) |
| Python Demo | [demo.py](../../scripts/ik_benchmark/scripts/demo.py) |
| Python Benchmark | [benchmark.py](../../scripts/ik_benchmark/scripts/benchmark.py) |

---

## 快速使用

### C++ 可执行文件

```bash
# 单臂 demo（默认 left_arm + pick_ik）
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver pick_ik/PickIkPlugin

# 单臂 TRAC-IK
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin

# 单臂 KDL
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver kdl_kinematics_plugin/KDLKinematicsPlugin

# 右臂
ros2 run alfa_robot_benchmarks ik_demo --group right_arm --solver pick_ik/PickIkPlugin

# 双臂
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver pick_ik/PickIkPlugin

# 无扰动模式（FK→IK 直接验证）
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver pick_ik/PickIkPlugin --no-perturb

# 批量 benchmark
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver pick_ik/PickIkPlugin --samples 50 --jsonl /tmp/result.jsonl
```

### Python 脚本

```bash
# Demo
python3 scripts/ik_benchmark/scripts/demo.py --group left_arm --solver pick_ik --no-perturb

# 多求解器对比 benchmark
python3 scripts/ik_benchmark/scripts/benchmark.py --groups left_arm right_arm --solvers kdl trac_ik pick_ik --samples 50
```

### C++ 库调用

```cpp
#include "ik_benchmark/ik_solver.h"

ik_benchmark::IkSolver ik("left_arm", "pick_ik/PickIkPlugin", /*timeout=*/2.0);

// 单臂 IK
Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
target.translation() = Eigen::Vector3d(0.3, 0.2, 0.5);
auto result = ik.solve(target);

// 正运动学
auto poses = ik.fk(ik.getHomeSeed());

// 随机 seed
auto seed = ik.getRandomSeed();
```

---

## 可用求解器

| 求解器 | 插件类名 | 单臂 | 双臂 | 说明 |
|--------|----------|------|------|------|
| pick_ik | `pick_ik/PickIkPlugin` | ✓ | ✓ | 默认，全局搜索 + memetic 优化 |
| TRAC-IK | `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` | ✓ | ✗ | 数值优化 + SQP，快速 |
| KDL | `kdl_kinematics_plugin/KDLKinematicsPlugin` | ✓ | ✗ | 运动学解析，适合简单构型 |

> 双臂组 (`dual_arm_with_base`) 目前仅支持 `pick_ik`。

---

## 命令行参数

### ik_demo

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--group` | `left_arm` | 规划组：left_arm / right_arm / dual_arm_with_base |
| `--solver` | `trac_ik.../TRAC_IKKinematicsPlugin` | IK 插件类名 |
| `--timeout` | `2.0` | 单次求解超时（秒） |
| `--trials` | `5` | 测试次数 |
| `--no-perturb` | off | 不对关节加扰动，直接 FK→IK |
| `--free-joint6` | off | 不固定 joint6 为 0 |

### ik_benchmark

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--group` | `left_arm` | 规划组 |
| `--solver` | `trac_ik.../TRAC_IKKinematicsPlugin` | IK 插件类名 |
| `--timeout` | `2.0` | 单次求解超时（秒） |
| `--samples` | `50` | 采样数 |
| `--perturb` | `0.1` | 关节扰动幅度（rad），0 = 无扰动 |
| `--free-joint6` | off | 不固定 joint6 为 0 |
| `--jsonl` | | JSONL 结果输出路径 |

---

## 关节组与变量

| 规划组 | 变量数 | 变量列表 |
|--------|--------|----------|
| `left_arm` | 8 | turn_link, updown_link, leftjoint1-6 |
| `right_arm` | 8 | turn_link, updown_link, rightjoint1-6 |
| `dual_arm_with_base` | 13 | turn_link, updown_link, leftjoint1-6, rightjoint1-6 |

> `left_arm` / `right_arm` 用 `<chain base_link="base_link">` 定义，运动链包含基座关节（turn_link, updown_link）。
> `joint6` 是 prismatic 吸盘关节，默认固定为 0。用 `--free-joint6` 可释放。

---

## IkSolver API 参考

```cpp
class IkSolver {
public:
    // 构造：指定组名、插件、超时、是否释放 joint6
    IkSolver(const std::string& group_name,
             const std::string& solver_plugin,
             double timeout = 2.0,
             bool free_joint6 = false);

    // 单臂 IK
    IkResult solve(const Eigen::Isometry3d& target,
                   const std::vector<double>& seed = {},
                   double timeout = 0.0);

    // 双臂 IK
    IkResult solveDual(const Eigen::Isometry3d& left_target,
                       const Eigen::Isometry3d& right_target,
                       const std::vector<double>& seed = {},
                       double timeout = 0.0);

    // 正运动学：单臂返回1个位姿，双臂返回2个
    std::vector<Eigen::Isometry3d> fk(const std::vector<double>& joint_values);

    // 全零 seed
    std::vector<double> getHomeSeed() const;

    // 随机 seed（joint6 默认固定为 0）
    std::vector<double> getRandomSeed() const;

    const std::vector<std::string>& getVariableNames() const;
    const std::string& getGroupName() const;
    bool isDualArm() const;
};

struct IkResult {
    bool success;
    std::vector<std::string> joint_names;   // 组内全部变量名
    std::vector<double> joint_values;       // 求解结果
    double solve_ms;                         // 求解耗时
    double pos_error;                        // 位置误差 (m)
    double ori_error;                        // 姿态误差 (rad)
};
```

### 实现细节

- **URDF 加载**：优先使用 `alfa_robot_description` 中预生成的 `.urdf`；若不存在则自动调用 `xacro` 生成到 `/tmp/`
- **SRDF 加载**：从 `alfa_robot_moveit_config/config/alfa_robot.srdf` 读取
- **IK 插件初始化**：通过 `pluginlib::ClassLoader` 动态加载，在 `rclcpp::Node` 上声明求解器所需参数
- **pick_ik 参数**：自动声明 mode, position/orientation_threshold, memetic 参数等
- **seed 处理**：使用 `RobotState::copyJointGroupPositions()` 获取完整变量向量（含基座关节）
- **FK 坐标系**：返回相对于 `base_link` 的末端位姿（通过 `getGlobalLinkTransform("base_link").inverse()` 变换）
- **误差计算**：位置误差为欧氏距离，姿态误差为角度差（AngleAxis）

---

## 测试结果

测试环境：ALFA 机器人 URDF/SRDF，50 次采样，0.1 rad 扰动，2.0s 超时

| 组 | 求解器 | 成功率 | 平均耗时 |
|----|--------|--------|----------|
| left_arm | pick_ik | 100% | ~50ms |
| left_arm | TRAC-IK | 100% | ~8ms |
| left_arm | KDL | 100% | ~3ms |
| right_arm | pick_ik | 100% | ~50ms |
| dual_arm_with_base | pick_ik | 100% | ~100ms |
