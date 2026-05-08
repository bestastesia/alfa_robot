# ALFA Robot IK 求解器

## 源码位置

| 文件 | 路径 |
|------|------|
| IK 求解器头文件 | [ik_solver.h](scripts/ik_benchmark/include/ik_benchmark/ik_solver.h) |
| IK 求解器实现 | [ik_solver.cpp](scripts/ik_benchmark/src/ik_solver.cpp) |
| 单次求解 demo | [demo_single.cpp](scripts/ik_benchmark/src/demo_single.cpp) |
| 批量 benchmark | [benchmark_main.cpp](scripts/ik_benchmark/src/benchmark_main.cpp) |
| Python benchmark | [benchmark.py](scripts/ik_benchmark/scripts/benchmark.py) |
| Python demo | [demo.py](scripts/ik_benchmark/scripts/demo.py) |

## 求解模式

| 求解器 | 快捷名 | 规划组 | 关节 | 方法 |
|--------|--------|--------|------|------|
| KDL | `kdl` | `left_arm` / `right_arm` | 6 DOF | `solve()` |
| TRAC-IK | `trac_ik` | `left_arm` / `right_arm` | 6 DOF | `solve()` |
| pick_ik | `pick_ik` | `dual_arm_with_base` | 13 DOF | `solveDual()` |
| bio_ik | `bio_ik` | `dual_arm_with_base` | 13 DOF | `solveDual()` |

- 位姿参考系: 单臂 `updown_link`，双臂 `base_link`
- 末端链接: 左臂 `leftjoint6_link`，右臂 `rightjoint6_link`
- 双臂模式需同时提供两个末端目标位姿

## 快速使用

```bash
cd ros2_ws && source install/setup.bash

# 单臂 demo
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver kdl

# 双臂 demo
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver pick_ik

# 批量 benchmark
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver kdl --samples 20
ros2 run alfa_robot_benchmarks ik_benchmark --group dual_arm_with_base --solver pick_ik --samples 10 --verbose

# Python 对比模式
python3 scripts/ik_benchmark/scripts/benchmark.py --mode compare --samples 20
```

---
---

> 以下为详细参考文档，供 AI Agent / 自动化工具使用。

## 编译

```bash
cd ros2_ws
colcon build --packages-select alfa_robot_description alfa_robot_moveit_config alfa_robot_benchmarks
source install/setup.bash
```

`scripts/ik_benchmark/` 通过 `ros2_ws/src/alfa_robot_benchmarks` 符号链接被 colcon 发现。
URDF 由 xacro 在 `alfa_robot_description` 包中生成，`ik_demo` / `ik_benchmark` 自动定位 `alfa_robot_description` 和 `alfa_robot_moveit_config` 的 share 目录。

## 已知限制：Prismatic 关节兼容性

**关键**: `leftjoint1` 和 `leftjoint6` 是 prismatic（滑台/伸缩）关节，不是 revolute。
这导致大部分 MoveIt IK 插件在 `left_arm` / `right_arm` 组（6 DOF）上求解困难或不可用：

| 求解器 | 单臂 (含 prismatic) | 双臂 (13 DOF) |
|--------|---------------------|---------------|
| KDL | 不兼容（JntArray 维度错误崩溃） | N/A |
| TRAC-IK | 不兼容 | N/A |
| pick_ik | 初始化成功但求解超时 | 初始化关节链不完整 |
| bio_ik | 初始化成功但求解失败 | 初始化关节链不完整 |

**根因**: IK 求解器通过 `JointModelGroup::getChain()` 提取从 base 到 tip 的运动链，但 prismatic 关节导致链结构与传统 6R 机器人不同，求解器内部算法不匹配。

**解决方案（待实现）**:
1. 将 prismatic 关节视为已知参数，仅对 4 个 revolute 关节求解
2. 或使用 `dual_arm_with_base` 组（含 updown），将 prismatic 关节位置作为 seed 约束
3. 或自定义 IK 插件，专门处理 prismatic+revolute 混合链

## C++ Demo (ik_demo)

随机生成关节角 → FK → 加噪 → IK → 输出结果

```bash
# 单臂
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver kdl
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver trac_ik
ros2 run alfa_robot_benchmarks ik_demo --group right_arm --solver trac_ik

# 双臂
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver pick_ik
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver bio_ik

# 调整扰动幅度
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver trac_ik --perturb-pos 0.03 --perturb-ori 0.15

# 人类可读输出（非 JSON）
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver kdl --no-json
```

输出为 JSON 格式，包含 `success`, `solve_ms`, `pos_error`, `ori_error`, `joint_names`, `joint_values`, `target_left`, `target_right`, `fk_left`, `fk_right` 等字段。

## C++ Benchmark (ik_benchmark)

多次采样，统计成功率、平均耗时、误差分布

```bash
# 单臂 20 样本
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver trac_ik --samples 20

# 双臂 10 样本 + 逐样本详细输出
ros2 run alfa_robot_benchmarks ik_benchmark --group dual_arm_with_base --solver pick_ik --samples 10 --verbose

# 导出 JSONL 数据文件
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver kdl --samples 50 --jsonl /tmp/result.jsonl

# 调整达标阈值
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver trac_ik --samples 100 --pos-thresh 0.003 --ori-thresh 0.005
```

统计输出包含：总样本数、成功(达标)、成功(误差大)、失败、平均/最大耗时、平均/最大位置误差、平均/最大姿态误差。

## Python Demo

```bash
# 单臂
python3 scripts/ik_benchmark/scripts/demo.py --group left_arm --solver trac_ik

# 双臂
python3 scripts/ik_benchmark/scripts/demo.py --group dual_arm_with_base --solver pick_ik
```

内部通过 subprocess 调用 C++ ik_demo，解析 JSON 输出后打印人类可读结果。

## Python Benchmark (对比模式)

```bash
# 对比所有单臂求解器 (kdl vs trac_ik)
python3 scripts/ik_benchmark/scripts/benchmark.py --mode compare --samples 20

# 对比所有双臂求解器 (pick_ik vs bio_ik)
python3 scripts/ik_benchmark/scripts/benchmark.py --mode compare --samples 10

# 指定单个求解器
python3 scripts/ik_benchmark/scripts/benchmark.py --group left_arm --solver trac_ik --samples 50

# 全部求解器一起跑
python3 scripts/ik_benchmark/scripts/benchmark.py --mode all --samples 20
```

`--mode compare` 会自动遍历对应模式的所有求解器（单臂: kdl + trac_ik; 双臂: pick_ik + bio_ik），汇总对比。

## 命令行参数

### ik_demo 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--group` | `left_arm` | 规划组: left_arm / right_arm / dual_arm_with_base |
| `--solver` | `kdl` | 求解器快捷名: kdl / trac_ik / pick_ik / bio_ik |
| `--perturb-pos` | `0.05` | 末端位置扰动幅度(m) |
| `--perturb-ori` | `0.25` | 末端姿态扰动幅度(rad) |
| `--timeout` | `2.0` | IK 求解超时(秒) |
| `--seed` | `42` | 随机种子 |
| `--no-json` | - | 输出人类可读格式而非 JSON |

### ik_benchmark 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--group` | `dual_arm_with_base` | 规划组 |
| `--solver` | `pick_ik` | 求解器快捷名 |
| `--samples` | `200` | 测试样本数 |
| `--timeout` | `2.0` | IK 求解超时(秒) |
| `--perturb-pos` | `0.05` | 末端位置扰动幅度(m) |
| `--perturb-ori` | `0.25` | 末端姿态扰动幅度(rad) |
| `--pos-thresh` | `0.005` | 位置误差达标阈值(m) |
| `--ori-thresh` | `0.01` | 姿态误差达标阈值(rad) |
| `--jsonl` | - | JSONL 输出文件路径 |
| `--verbose` | - | 逐样本详细输出 |

## C++ API

```cpp
#include <ik_benchmark/ik_solver.h>

// 创建求解器 — 需要显式指定 URDF/SRDF 路径和插件全名
ik_benchmark::IkSolver solver(
    urdf_path, srdf_path,
    "left_arm",                                              // 规划组
    "pick_ik/PickIkPlugin",                                  // 求解器插件全名
    2.0                                                      // 默认超时(秒)
);

// 单臂 IK — 返回 IkResult
auto result = solver.solve(target_pose);       // target_pose: Eigen::Isometry3d
// result.success, .solve_ms, .pos_error, .ori_error, .joint_names, .joint_values

// 双臂 IK — 需同时提供左右末端目标
auto result = solver.solveDual(left_target, right_target);

// 正运动学 — 返回 vector<Isometry3d>，单臂1个，双臂2个
auto poses = solver.fk(joint_values);

// 随机关节种子（在关节限制内）
auto seed = solver.getRandomSeed();

// 零位种子
auto home = solver.getHomeSeed();

// 判断是否双臂模式
bool dual = solver.isDualArm();
```

### IkResult 结构

```cpp
struct IkResult {
    bool success;                        // 是否求解成功
    std::vector<std::string> joint_names; // 关节名列表
    std::vector<double> joint_values;     // 求解关节值
    double solve_ms;                      // 求解耗时(ms)
    double pos_error;                     // 位置误差(m)
    double ori_error;                     // 姿态误差(rad)
};
```

## 外部调用方式

1. **命令行**: `ros2 run alfa_robot_benchmarks ik_demo --group <组> --solver <求解器>` — stdout 输出 JSON
2. **C++ 链接**: `find_package(alfa_robot_benchmarks)` → `#include <ik_benchmark/ik_solver.h>` → 链接 `ik_solver_lib`
3. **Python**: subprocess 调用 ik_demo，解析 JSON 输出

## 文件结构

```
scripts/ik_benchmark/
├── include/ik_benchmark/
│   └── ik_solver.h           # IkSolver 类声明
├── src/
│   ├── ik_solver.cpp          # IkSolver 实现 (RobotModelLoader + pluginlib)
│   ├── benchmark_main.cpp     # ik_benchmark 可执行文件
│   └── demo_single.cpp        # ik_demo 可执行文件
├── scripts/
│   ├── benchmark.py           # Python 批量对比脚本
│   └── demo.py                # Python 单次 demo 脚本
├── CMakeLists.txt
└── package.xml
```

## 依赖

| 依赖 | 说明 |
|------|------|
| `moveit_core` | RobotModel, RobotState, KinematicsBase |
| `moveit_ros_planning` | RobotModelLoader |
| `pluginlib` | 动态加载 IK 插件 |
| `srdfdom` + `urdf` | 解析 URDF/SRDF |
| `alfa_robot_description` | URDF/SRDF 文件 (exec_depend) |
| `alfa_robot_moveit_config` | SRDF + kinematics.yaml (exec_depend) |
| `pick_ik` (可选) | 双臂 IK 插件 |
| `trac_ik` (可选) | 单臂 IK 插件 |
| `bio_ik` (可选) | 双臂 IK 插件 |
