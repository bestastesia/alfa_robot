# V3 Demo 启动速查

只看用途和操作。以下均为**仿真，不控制实机**；本机工作区已编译时直接使用。

## 0. 每个新终端都先执行（启动、二次调用都不能省）

```bash
cd ~/Sevenova/alfa_robot
source tools/ros_humble_env.sh
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=188
```

- **终端 A**：执行下面选中的一条启动命令，保持运行。
- **终端 B**：也先执行上面四行，再执行对应“二次操作”；不用重新 launch。
- **关闭后重开**：重新执行环境命令和启动命令，不是只发 service。
- 本页统一使用 domain 188。同一时间只开一个 Demo；切换前在 A 按 `Ctrl+C`。已在其他 domain 启动的实例不会收到这里的请求。

## 1. 仓库单箱抓取：固定肩高偏移版

**用途**：输入车头到箱墙的距离和箱号，先升降到肩心高于箱体中心 `0.25m`，再接近、吸取、抽出、后置并释放。包含仓库和邻箱碰撞检查；默认采用远端主编排。

**首次启动（A）**：启动后自动计算一次。

```bash
ros2 launch alfa_robot_moveit_config v3_box_wall_grasp_demo.launch.py \
  x:=0.90 box_id:=5 arm:=auto
```

**二次操作（B）**：修改字段即可重新规划，不重启窗口。

```bash
ros2 service call /v3_box_wall_grasp_demo/plan_wall_box \
  alfa_robot_moveit_config/srv/PlanWallBoxDemo \
  '{x: 0.90, box_id: 5, arm: auto}'
```

`x` 单位米；`box_id` 为0～24，底层0～4，向上依次编号，每层沿世界 +Y 递增；`arm` 为 `left/right/auto`（auto先左后右）。每次是独立场景请求，**不是连续拆掉上一箱后的箱墙**。

## 2. 仓库单箱抓取：单次舒适高度实验版

**用途**：将第1版的固定 `0.25m` 偏移换成“肩心到吸取TCP的距离比例”选高；先检查当前姿态的无碰撞升降范围，再选最接近候选区间的高度；每臂只选一次，其余流程相同。默认候选比例 `[1.10,1.15]`、首选 `1.15`；**独立验证没有证明优于旧版**。

**首次启动（A）**：与第1版二选一，启动后自动计算一次。

```bash
ros2 launch alfa_robot_moveit_config v3_box_wall_comfort_grasp_demo.launch.py \
  x:=0.90 box_id:=5 arm:=auto
```

**二次操作（B）**：服务名和第1版相同，使用哪个策略由 A 启动的版本决定。

```bash
ros2 service call /v3_box_wall_grasp_demo/plan_wall_box \
  alfa_robot_moveit_config/srv/PlanWallBoxDemo \
  '{x: 0.90, box_id: 5, arm: auto}'
```

## 3. 单臂自由箱位抽取（历史研究入口）

**用途**：在 RViz 拖动单个箱子的 XYZ，验证接近、吸取、抽出和返回；不是当前主编排。完整旧实现已冻结，当前需要携箱回 Home 时优先使用第1节入口的 `wall_context:=target_only post_extract_policy:=loaded_home`。

**首次启动（A）**：

```bash
ros2 launch alfa_robot_moveit_config v3_single_arm_box_extract_demo.launch.py
```

**二次操作**：RViz 选 `Interact`，拖动箱体控制球，右键确认计算；或调整后在 B 调用：

```bash
ros2 service call /v3_single_arm_box_extract_demo/run_current_box std_srvs/srv/Trigger '{}'
```

## 4. 冗余解析 IK 解族查看

**用途**：查看同一末端位姿对应的多组关节解；**不检查碰撞，不是可执行抓取轨迹**。

**首次启动（A）**：右臂改为 `side:=right`。

```bash
ros2 launch alfa_robot_moveit_config v3_redundant_ik_interactive_demo.launch.py side:=left
```

**二次操作**：RViz 选 `Interact`，拖动/旋转蓝色末端目标；稳定后自动重算，Rerun刷新。**没有二次 service 命令，也不用重启。**

## 5. 双臂搬箱：三个模式共用一套二次命令

**用途**：从已有握持姿态出发验证双臂同步运动，不包含从箱墙接近吸取的全过程。

**首次启动（A，下面三条只选一条）**：

```bash
# 对称握持平移：两臂夹住40cm箱体，固定朝向沿直线搬运
ros2 launch alfa_robot_moveit_config v3_dual_arm_cartesian_box_demo.launch.py

# 绕箱心旋转：夹住30cm箱体，中心不动，绕世界X轴旋转
ros2 launch alfa_robot_moveit_config v3_dual_arm_box_roll_demo.launch.py

# 异构握持平移：30cm箱体，默认右臂侧握、左臂从底面向上支撑
ros2 launch alfa_robot_moveit_config v3_dual_arm_asymmetric_box_demo.launch.py
```

**二次操作**：RViz 选 `Interact`，平移模式拖青色球、旋转模式拖蓝色旋转环，右键确认计算；也可调整后在 B 调用（三个模式服务名确实相同）：

```bash
ros2 service call /v3_dual_arm_cartesian_box_demo/run_current_target std_srvs/srv/Trigger '{}'
```

## 6. 只记住这两种报错

| 现象 | 处理 |
|---|---|
| `The passed service type is invalid` | 在**调用服务的终端**重做第0节；用 `ros2 interface show alfa_robot_moveit_config/srv/PlanWallBoxDemo` 检查接口是否加载。 |
| `waiting for service to become available` | 确认 A 仍在运行、服务名对应所启动的 Demo，且 A/B 都执行了第0节，domain和本机通信设置一致。 |

本页不重复实验过程；算法、参数扫描与验收记录留在各 Demo 的详细文档中。
