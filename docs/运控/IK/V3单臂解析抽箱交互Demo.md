# V3单臂解析抽箱交互Demo

## 目标

验证 V3.1.1 左臂完成以下固定流程。该交互入口仅作历史研究；当前主编排及本地策略兼容方式见 [V3.1.1远端流程整合说明](V3.1.1远端流程整合说明.md)。

1. 使用 RRTConnect 从初始关节位移动到箱体正面前 5cm；
2. 使用七轴冗余解析 IK 沿笛卡尔直线前进 5cm；
3. 将 `0.30m × 0.40m × 0.40m` 箱体附着到 `left_tool0`；
4. 使用解析 IK 沿笛卡尔直线向机器人方向抽出 35cm；
5. 携带箱体使用 RRTConnect 返回初始关节位。

目标箱上下左右各有一个同尺寸静态邻箱。静态邻箱、机器人、自碰撞和附着箱使用同一份
MoveIt PlanningScene/FCL 检查。为了避免理想几何恰好共面被数值判为穿透，碰撞尺寸默认
向内收 `2mm/面`，Rerun/RViz仍按真实箱体尺寸显示。

## 启动

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alfa_robot_moveit_config v3_single_arm_box_extract_demo.launch.py
```

启动后：

1. 在 RViz 使用箱堆前外侧青色控制球的红绿蓝平移轴调整目标箱 XYZ；青色连线指向实际目标箱；
2. Rerun 会同步显示目标箱、四个邻箱、预接触点、接触点和抽出终点；
3. 右键青色控制球，选择 **确认并计算当前箱位**；
4. 终端和 Rerun 会显示计算开始、总耗时、分阶段耗时和失败原因；
5. 成功时自动播放完整轨迹，失败时播放已经形成的合法前缀并显示失败阶段。

也可以不用右键菜单，直接触发当前箱位：

```bash
ros2 service call /v3_single_arm_box_extract_demo/run_current_box std_srvs/srv/Trigger '{}'
```

## 自动烟测

不打开窗口并自动计算默认箱位：

```bash
ros2 launch alfa_robot_moveit_config v3_single_arm_box_extract_demo.launch.py \
  start_rviz:=false start_rerun:=false auto_run_once:=true
```

## 算法边界

- 长距离自由空间运动：OMPL `RRTConnect`，使用完整机器人和任务场景碰撞。
- 5cm接触和35cm抽出：每1cm生成一个固定末端位姿，七轴解析IK遍历冗余角并选取连续、
  远离限位且碰撞合法的分支；相邻解析状态额外做2.5度关节插值碰撞检查。
- 箱体从接触结束开始附着到末端；35cm抽出和返回阶段均把箱体作为机器人一部分检查。
- 当前 Demo 固定末端朝世界 `+X`，只允许用户调整箱体中心 `X/Y/Z`。

## 2026-09-11：mentor 任务的前置开发（独立仿真，不接实机）

用户已确认 mentor 允许直接在旧架构本 demo 上开发完整任务。新架构学习继续，
但不再以缺少新架构视频工作树阻塞本任务。本节只完成场景和回归基础，**不是双箱抽取、放置验收**。
本轮基于 `alfa_v3_dev` / `1e6c58f` 的工作树，已有环境与冗余 IK 修复保留，未提交或推送。

### 当前调用链与复用边界

```text
launch + demo_config YAML
  → init：V3.1.1 模型、7轴活动臂、固定共享轴、场景参数
  → ~/run_current_box (Trigger)：仅接受计算请求
  → planTask
      makeScene → 检查起点与携箱回程终点
      solvePoseCandidates → traceCartesianPath（接触、附着、抽离）
      planRrt（到预接触、携箱回初始位）
  → ~/task_json：preview / planning / result + success/failure_stage/reason/frames
  → ~/joint_states、~/scene_markers → robot_state_publisher / RViz
  → alfa_robot_rerun/v3_single_arm_box_extract_viewer.py：只读回放
```

可复用现有 IK、RRT、附着箱与场景适配，不复制求解器、不新建 ROS 包。
Rerun 已按 `neighbor_centers` 数组绘制，不限定四个邻箱，因此本次不改 viewer。
`result.success` 仅代表这条单臂规划链成功；最后仍携箱，没有释放与放置。
回放关节没有真实执行确认，也不是符合动力学/时间约束的双臂控制轨迹。

### 距离交付前的工作清单

| 工作 | 本轮状态 | 下一道检查 |
| --- | --- | --- |
| 核对旧仓规范、入口、已有改动 | 已完成；RTK 文件缺失，见下文 | 获得实际适用的 RTK 或确认该引用过期 |
| YAML 参数入口，保留原单箱模式 | 已实现 `demo_config` | 原单箱链回归 |
| 5×5 墙、1cm 横纵间隙、选择目标但不移动墙 | 已实现；单臂一次只选择一个目标 | 几何/marker/JSON 一致性检查，不承诺可达 |
| 16轴状态和回放 | 已实现；原14臂轴顺序不变，尾部追加 updown/head_joint=0 | 完整关节消息与回放检查 |
| 输入检查与 reset | 行列越界、非正采样步长、非法内缩拒绝；reset 回到配置位置 | 非法输入必须在任务发布前失败 |
| 距离交付口径 | 已列出，等待同事填实测值 | 坐标系、基准面、姿态与模型必须一致 |
| 接触前目标箱碰撞、完整机器人碰撞审计 | 未修改；发现基线缺口 | 应作为双箱开发前的下一项，不等距离也能做 |
| 双臂规划组、两负载与共享轴策略 | 调用边界已查清，尚未实现 | 不可将两个单臂成功拼成双臂成功；检查臂间、箱间碰撞 |
| 同排/差一排配对、开洞后的场景更新 | 验收维度已列出，尚未实现 | 确认覆盖集合与逐次取箱顺序 |
| 后放、释放、空载回家 | 尚未实现 | 指定放置位姿/容差、支撑面与释放判据 |
| 实测距离的可达性与全流程 Rerun 录制 | 等同事结果 | 使用同一场景参数，保存成功和失败，不只挑成功案例 |

### 场景输入约定（demo fixture，不是生产任务接口）

- `scene_layout: cross` 为旧默认：目标箱＋四邻箱、无几何间隙，默认碰撞内缩 2mm/面。
- `scene_layout: wall_5x5` 为新增：目标＋24邻箱，`wall_gap` 默认 `0.01m`，横纵相同；
  默认 `collision_inset=0`，禁止把内缩伪装成实际间距。
- `wall_origin` **必填**：`world` 中第0行第0列箱体的**中心**坐标，单位米。
  行从底部0向 `+Z` 增至4，列沿 `+Y` 从0增至4，不使用“画面左/右”作坐标定义。
- `wall_target_row/column` 只选目标，不改整墙位置；
  `center(r,c)=origin+[0,c*(width+gap),r*(height+gap)]`。
  箱体正面朝 `-X`，正面中心 X 为 `origin.x-depth/2`，机械臂沿 `+X` 接触、`-X` 抽出。
- `initial_box_center` 仅用于 cross；wall 模式由 origin 和目标行列计算。
- RViz 控制球仍用于实验性整体平移：拖动目标会平移整墙；reset 恢复文件配置。
  正式固定场景验收不得边运行边拖动。
- `side` 决定左/右解析模型，planning_group/tool_link 如显式提供，必须与 side 一致；
  本 demo 的 arm_base_link 固定为 `arm_carriage`，world_frame 必须等于模型根坐标系。
- 当前继承原 demo 的双臂零关节起点，升降/头部均为0；**未切换为 SRDF home**。
  `NOW.md` 的统一 home 与它不一致，需先确认距离实验采用哪种姿态，再统一。

给同事的距离交付字段（不要只给一个裸数）：

```text
模型版本/commit；使用的配置文件
距离数值及单位；从底盘哪个点/面到箱墙哪个点/面；方向/坐标系
底盘在 world 下的位姿；updown、head_joint、双臂初始关节
箱体 depth/width/height；横纵净间隙；底层中心高度与横向偏置
测量/扫描覆盖的目标集合；“合适”的判据与对应结果文件
```

如果交付的是沿 world +X 从底盘前表面 `x_front` 到箱墙近表面的净距 `d`，
则 `wall_origin.x = x_front + d + box_depth/2`；如果交付的是其他基准，不能套这个式子。
底层落地且地面为 `z=0` 才能用 `origin.z=height/2`。本次没有猜测实际距离。

### 可重复检查

从本机仓库根目录执行；只启动专用规划/状态发布 demo，不启动 controller、PLC 或实机节点：

```bash
cd /home/astesia/Sevenova/alfa_robot
source tools/ros_humble_env.sh
cd ros2_ws
colcon build --packages-select alfa_robot_moveit_config \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
source install/setup.bash
ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/alfa_robot_moveit_config/test/test_v3_box_wall_preparation.py \
  --artifacts /tmp/v3_box_wall_preparation
```

先确认 domain 182 没有其他实验。脚本关闭界面、保存每个用例参数/消息/日志、结束时仅清理自己启动的进程组。
测试包括旧单箱四段规划、固定墙不同目标的几何一致性、16轴消息/回放、不可达箱墙目标的规划拒绝及非法参数拒绝。
几何用例用的是**合成坐标**，不是同事测定的距离，也不是25箱可达性证明。

例如仅预览箱墙（参数文件为本地实验输入，不将示例坐标当作验收配置）：

```yaml
v3_single_arm_box_extract_demo:
  ros__parameters:
    scene_layout: wall_5x5
    wall_origin: [1.23, -0.82, 0.20]  # 合成几何用例；必须替换为实测换算坐标
    wall_target_row: 0
    wall_target_column: 2
    wall_gap: 0.01
```

将它保存为 `/tmp/wall_preview.yaml`，在已 source 的终端运行：

```bash
ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 ros2 launch alfa_robot_moveit_config \
  v3_single_arm_box_extract_demo.launch.py demo_config:=/tmp/wall_preview.yaml
```

默认不自动计算。可用 `auto_run_once:=true` 请求一次规划，但失败应记录原始原因，不能通过放宽几何或关闭碰撞强行变绿。

### 缺失规范、接口与下一阶段约束

1. `AGENTS.md` 引用 `/home/li/.codex/RTK.md`，本机缺失。其余 START/NOW/TASKS、运控岗位、
   包职责和组织 Git 规范可读。请提供实际适用文件或确认该绝对路径引用过期。
2. 当前对话授权的是旧 demo 仿真开发，不是把它注册为正式 runtime 服务。
   按 `系统架构与包职责边界.md`，正式场景要有 scene_id，正式任务由 runtime 持有，viewer 只读；
   本 demo 的 preview/generation 不等价于生产 scene_id/state_id。本次未改正式接口/启动链。
3. `~/run_current_box` 是空请求 Trigger；没有双箱位姿、放置目标、取消/超时或执行完成反馈。
   仓库虽有 RunDualArmPoseTask/RunDualGraspTask 等协议，不能只凭消息存在就宣称已接通本 demo。
   行列 ID 仅用于仿真 fixture，不能绕过 NOW 中正式任务以左右正面中心 pose_6d 输入的要求。
4. `makeScene` 当前只放邻箱，目标从接触后才附着，故接触前机器人与目标箱碰撞没有闭环；
   touch_links 也沿用原范围。双箱前应补充目标 world→attached→world 生命周期、精确接触白名单，
   并审计 `scene_collision_reason` 的 planning_group 过滤是否覆盖非活动臂。
   不能把“24个障碍已加入”解释为整个抓放过程碰撞安全。
5. 固定底盘不自动等于固定升降/头部。共享轴能否参与规划、双臂同步/错时规则、后放目标
   （含支撑面和容差）、箱子顺序/配对覆盖仍须在验收配置中明确。
6. 本任务的 Linear issue、验收人和最终分支/PR 归属尚未给出。没有冒用 MOTION-94/154，
   没有自动创建 issue、提交、推送或改同事的距离实验；本轮保留在原工作树供审阅。

### 本机验证结果（2026-09-11）

- Release 单包构建成功；新增检查脚本 **10/10**：默认单箱成功，三个墙内目标几何检查通过，
  一个远距离目标返回 `precontact_ik` 失败，五个非法配置被拒绝。
- 既有 Rerun viewer 无窗口消费箱墙 preview，生成 `wall_geometry_preview.rrd`；
  `rerun rrd verify` 通过，`rrd print` 中确实存在 `/world/boxes/neighbors` 数据。
  这是几何预览记录，未人工检查 GUI，也不是抓放动作录制。
- 证据目录：`/home/astesia/Sevenova/日志/验收_2026-09-11/old_demo_wall_preparation/`，
  包含 build/tests 日志、每用例参数和收到的 JSON、Rerun 日志/校验输出。
- 没有验证双箱规划、25箱覆盖、放置/释放或真实执行；未改变同事的距离结果。

### 距离输入与箱号服务版本（2026-09-11）

新增独立 launch `v3_box_wall_grasp_demo.launch.py`，复用本 demo 的实现，接受车头到墙距离 `x`、`box_id` 与 `arm`，补充目标箱接触前/附着后碰撞及 RViz 携箱回放。完整使用方法、编号、模型假设、验证与缺失合同见同目录 `V3距离输入单箱抓取Demo.md`；旧交互入口默认行为不变。
