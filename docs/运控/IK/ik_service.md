# IK 求解服务

本文只记录当前仍保留的 IK 调用方式。历史验证过的 `pick_ik`、`trac_ik` 已从工作区清理；当前主线只保留：

- 单臂链式 IK：`kdl_kinematics_plugin/KDLKinematicsPlugin`
- 双臂/双末端实验 IK：`bio_ik/BioIKKinematicsPlugin`
- 当前工程化双臂流程：以自研候选筛选、抽离规划和负重规划为主，MoveIt IK 插件只作为底层能力之一

## 源码位置

| 内容 | 路径 |
|------|------|
| IK 求解器封装 | `scripts/ik_benchmark/include/ik_benchmark/ik_solver.h` / `scripts/ik_benchmark/src/ik_solver.cpp` |
| Demo 脚本 | `scripts/ik_benchmark/scripts/demo.py` |
| Benchmark 脚本 | `scripts/ik_benchmark/scripts/benchmark.py` |
| 固定轴可达性网格 | `scripts/ik_benchmark/scripts/ik_range_grid.py` / `scripts/ik_benchmark/src/range_grid_main.cpp` |
| 当前双臂全流程 | `ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp` |

## 快速使用

### 单臂 KDL

```bash
python3 scripts/ik_benchmark/scripts/demo.py \
  --group left_arm \
  --solver kdl \
  --no-perturb
```

```bash
python3 scripts/ik_benchmark/scripts/benchmark.py \
  --groups left_arm right_arm \
  --solvers kdl \
  --samples 50
```

### 双臂 BioIK

```bash
python3 scripts/ik_benchmark/scripts/demo.py \
  --group dual_arm_with_base \
  --solver bio_ik
```

```bash
python3 scripts/ik_benchmark/scripts/benchmark.py \
  --groups dual_arm_with_base \
  --solvers bio_ik \
  --samples 50
```

### 直接指定 MoveIt IK 插件

```bash
python3 scripts/ik_benchmark/scripts/demo.py \
  --group left_arm \
  --solver kdl_kinematics_plugin/KDLKinematicsPlugin
```

```bash
python3 scripts/ik_benchmark/scripts/demo.py \
  --group dual_arm_with_base \
  --solver bio_ik/BioIKKinematicsPlugin
```

## 可用求解器

| 求解器 | 插件类名 | 单臂 | 双臂/双末端 | 当前用途 |
|--------|----------|------|-------------|----------|
| KDL | `kdl_kinematics_plugin/KDLKinematicsPlugin` | ✓ | ✗ | 单臂固定链 IK、九向可达性、抽离单步 IK |
| BioIK | `bio_ik/BioIKKinematicsPlugin` | ✓ | ✓ | 双末端候选 IK、离散 h × 多 seed × cost scorer |

> `pick_ik` 和 `trac_ik` 曾用于早期对比，但现在不是主线依赖；仓库内源码和默认脚本入口已清理，避免误以为还能直接运行。

## 关节组与变量

| 规划组 | 变量数 | 变量列表 |
|--------|--------|----------|
| `left_arm` | 8 | `turn`, `updown`, `leftjoint1-6` |
| `right_arm` | 8 | `turn`, `updown`, `rightjoint1-6` |
| `dual_arm_with_base` | 14 | `turn`, `updown`, `pitch`, `leftjoint1-6`, `rightjoint1-6` |

实际变量数以当前 SRDF/URDF 为准；新机械臂命名已尽量去掉 `v5` 这类版本前缀。

## IkSolver API 参考

```cpp
class IkSolver {
public:
    IkSolver(const std::string& group_name,
             const std::string& solver_plugin,
             double timeout = 2.0,
             bool free_joint6 = false);

    IkResult solve(const Eigen::Isometry3d& target,
                   const std::vector<double>& seed = {},
                   double timeout = 0.0);

    IkResult solveDual(const Eigen::Isometry3d& left_target,
                       const Eigen::Isometry3d& right_target,
                       const std::vector<double>& seed = {},
                       double timeout = 0.0);

    std::vector<Eigen::Isometry3d> fk(const std::vector<double>& joint_values);
    std::vector<double> getHomeSeed() const;
    std::vector<double> getRandomSeed() const;

    const std::vector<std::string>& getVariableNames() const;
    const std::string& getGroupName() const;
    bool isDualArm() const;
};
```

## 实现注意

- URDF/SRDF 从 `alfa_robot_description` 与 `alfa_robot_moveit_config` 加载。
- IK 插件通过 `pluginlib::ClassLoader` 动态加载。
- KDL 适合单臂链；双臂组不是链式结构，不应指望 KDL 直接解双末端。
- BioIK 仍有随机性，因此主线双臂流程需要候选保留、去重、代价排序和后续抽离/负重规划验证。
- 当前正式流程优先参考 `docs/运控/工程化护栏/新机械臂命名与仿真服务化说明.md`。
