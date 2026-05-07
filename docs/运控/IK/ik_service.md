# ALFA Robot IK 求解器

## 求解模式

### 单臂 (6 DOF)

| 求解器 | 快捷名 | 规划组 | 关节 |
|--------|--------|--------|------|
| KDL | `kdl` | `left_arm` / `right_arm` | joint1-6 |
| TRAC-IK | `trac_ik` | `left_arm` / `right_arm` | joint1-6 |

### 双臂 (1+6+6 = 13 DOF)

| 求解器 | 快捷名 | 规划组 | 关节 |
|--------|--------|--------|------|
| pick_ik | `pick_ik` | `dual_arm_with_base` | updown + leftjoint1-6 + rightjoint1-6 |
| bio_ik | `bio_ik` | `dual_arm_with_base` | updown + leftjoint1-6 + rightjoint1-6 |

- 位姿参考系: `base_link`
- 末端链接: 左臂 `leftjoint6_link`，右臂 `rightjoint6_link`
- 双臂模式需同时提供两个末端目标位姿

## 快速使用

### C++ Demo (命令行)

```bash
source install/setup.bash

# 单臂
ros2 run alfa_robot_benchmarks ik_demo --group left_arm --solver kdl
ros2 run alfa_robot_benchmarks ik_demo --group right_arm --solver trac_ik

# 双臂
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver pick_ik
ros2 run alfa_robot_benchmarks ik_demo --group dual_arm_with_base --solver bio_ik
```

### C++ Benchmark (批量测试)

```bash
# 单臂 20 样本
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver trac_ik --samples 20

# 双臂 10 样本
ros2 run alfa_robot_benchmarks ik_benchmark --group dual_arm_with_base --solver pick_ik --samples 10 --verbose

# 输出 JSONL
ros2 run alfa_robot_benchmarks ik_benchmark --group left_arm --solver kdl --samples 50 --jsonl /tmp/result.jsonl
```

### Python Demo

```bash
python3 scripts/ik_benchmark/scripts/demo.py --group left_arm --solver trac_ik
python3 scripts/ik_benchmark/scripts/demo.py --group dual_arm_with_base --solver pick_ik
```

### Python Benchmark (对比)

```bash
# 对比所有单臂求解器
python3 scripts/ik_benchmark/scripts/benchmark.py --mode compare --samples 20

# 指定单个
python3 scripts/ik_benchmark/scripts/benchmark.py --group left_arm --solver trac_ik --samples 20
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--group` | `left_arm` | 规划组名 |
| `--solver` | `kdl` | 求解器: kdl / trac_ik / pick_ik / bio_ik |
| `--samples` | `200` | 测试样本数 |
| `--timeout` | `2.0` | IK 超时(秒) |
| `--perturb-pos` | `0.05` | 末端位置扰动幅度(m) |
| `--perturb-ori` | `0.25` | 末端姿态扰动幅度(rad) |
| `--seed` | `42` | 随机种子 |

## C++ API

```cpp
#include <ik_benchmark/ik_solver.h>

// 创建求解器
ik_benchmark::IkSolver solver(
    urdf_path, srdf_path,
    "left_arm",                       // 规划组
    "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",  // 求解器插件
    2.0                               // 默认超时
);

// 单臂 IK
auto result = solver.solve(target_pose);  // Eigen::Isometry3d

// 双臂 IK
auto result = solver.solveDual(left_target, right_target);

// 正运动学
auto poses = solver.fk(joint_values);  // 单臂1个, 双臂2个
```

## 外部调用方式

1. **命令行**: `ros2 run alfa_robot_benchmarks ik_demo --group <组> --solver <求解器>` — stdout 输出 JSON
2. **C++ 链接**: `find_package(alfa_robot_benchmarks)` → `#include <ik_benchmark/ik_solver.h>` → 链接 `ik_solver_lib`
3. **Python**: `subprocess` 调用 ik_demo，解析 JSON 输出

## 依赖

| 依赖 | 说明 |
|------|------|
| `moveit_core` | RobotModel, RobotState, KinematicsBase |
| `pluginlib` | 动态加载 IK 插件 |
| `srdfdom` + `urdf` | 解析 URDF/SRDF |
| `alfa_robot_description` | URDF/SRDF 文件 |
| `alfa_robot_moveit_config` | SRDF 配置 |
| `pick_ik` (可选) | 双臂 IK 插件 |
| `trac_ik` (可选) | 单臂 IK 插件 |
| `bio_ik` (可选) | 双臂 IK 插件 |
