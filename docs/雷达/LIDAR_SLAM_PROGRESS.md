# 雷达SLAM项目进度汇报

> 调查日期: 2026-05-07
> 基于对 `/home/ar/` 下所有相关工作空间的文件遍历和代码阅读

---

## 一、工作空间总览

| 工作空间 | 路径 | ROS版本 | 性质 | 最近修改 |
|---------|------|---------|------|---------|
| **fast_lio_ws** | `/home/ar/fast_lio_ws/` | ROS2 | **SLAM主开发空间** | 2026-04-30 |
| **alfa_robot** | `/home/ar/lhy_dev/alfa_robot/` | ROS2 | **项目主仓库** | 2026-04-28(打包) |
| **FastLio** | `/home/ar/FastLio/` | ROS2 | FastLio独立版 | 2026-04-29 |
| testlidarV2 | `/home/ar/testlidarV2/` | ROS1(catkin) | 早期测试(已废弃) | 2026-03-10 |
| testlidar_V2 | `/home/ar/testlidar_V2/` | ROS1(catkin) | 早期测试(已废弃) | 2026-02-07 |

---

## 二、模块详细说明

### 2.1 Fast-LIO2 — 3D LiDAR-Inertial SLAM

**位置**:
- `fast_lio_ws/src/FAST_LIO_LOCALIZATION2/` (含重定位功能)
- `alfa_robot/src/fast_lio/` (主项目集成版)
- `/home/ar/FastLio/` (独立版, 被fast_lio_ws软链接引用)

**核心能力**:
- 紧耦合LiDAR-IMU里程计与建图 (基于IEKF + ikd-Tree)
- 输入: `/livox/lidar` + `/livox/imu`
- 输出: `/cloud_registered` (配准点云), `/Odometry` (里程计), TF(`camera_init→body`)
- 支持Livox Avia / Mid-360 / Velodyne / Ouster

**当前配置 (mid360.yaml)**:
```yaml
lidar_type: 1          # Livox
scan_line: 4           # 4线
fov_degree: 360.0      # 全向
det_range: 100.0       # 探测距离100m
gravity_alignment: true             # 重力对齐已开启
extrinsic_est_en: true             # 外参在线估计已开启
pcd_save_en: true                   # PCD保存已开启
```

**alfa_robot版本的额外特性**:
- `fov_filter_node.cpp`: 视场角过滤器, 按D455相机FOV(90°×65°)裁剪点云, 用于雷达-视觉协同
- `mapping_mid360_align.launch.py`: 支持ICP初始对齐的建图
- `pointcloud_to_laserscan.launch.py`: 3D点云转2D激光扫描

**完成度**: ★★★★★ 核心算法成熟, 配置已调优, 已在主项目编译集成

---

### 2.2 FAST_LIO_LOCALIZATION2 — 全局重定位

**位置**: `fast_lio_ws/src/FAST_LIO_LOCALIZATION2/`

**核心文件**:
| 文件 | 功能 |
|------|------|
| `global_localization.py` | Open3D ICP多尺度全局匹配(粗→精, scale=5→1), FOV裁剪, fitness阈值判断 |
| `transform_fusion.py` | 坐标变换融合(map↔odom) |
| `publish_initial_pose.py` | 初始位姿发布 |
| `invert_livox_scan.py` | Livox扫描反转工具 |
| `localization.launch.py` | 一键启动: fast_lio + 全局定位 + PCD地图发布 + TF融合 |

**定位流程**:
1. 加载预建PCD地图
2. 接收当前帧配准点云(`/cloud_registered`)
3. 在全局地图中裁剪FOV范围内子地图
4. 多尺度ICP匹配求取`T_map_to_odom`
5. 发布修正后的Odom(`/map_to_odom`)

**完成度**: ★★★★☆ 功能完成, launch和Python脚本4月28日有更新

---

### 2.3 lio_2d_mapper — 2D栅格地图与导航桥接 [当前开发重点]

**位置**: `fast_lio_ws/src/lio_2d_mapper/`

**这是SLAM→导航的关键桥梁, 包含5个子节点**:

| 节点 | 源文件 | 功能 | 最后修改 |
|------|--------|------|---------|
| `mapper_node` | `mapper_node.cpp` | 3D点云→2D OccupancyGrid (Bresenham射线追踪 + log-odds概率更新), 20Hz在线发布 | **2026-04-30** |
| `lidar_tf_node` | `lidar_tf_node.cpp` | 3D→2D TF投影: body的3D位姿投影为2D(只保留yaw), 发布`map→2d_body` | 2026-04-27 |
| `lidar_init_pose_node` | `lidar_init_pose_node.cpp` | AMCL自动初始化: 从`/Odometry`获取位姿→发布`/initialpose` | 2026-04-23 |
| `pcd_to_grid_map_node` | `pcd_to_grid_map_node.cpp` | 离线PCD→2D地图: PassThrough+RadiusOutlier滤波→pgm/yaml | 2026-04-29 |
| `offline_map_publisher` | `offline_map_publisher.cpp` | 离线地图发布: 读取pgm/yaml→周期发布`/offline_map` | 2026-04-27 |

**mapper_node核心算法**:
- 订数模型: log-odds概率更新 (hit=+0.9, miss=-0.1)
- 射线追踪: Bresenham算法标记自由空间
- 衰减机制: 每帧对全图施加decay_factor=0.99的慢衰减
- 阈值: p>0.65为障碍, p<0.35为自由, 其余未知
- 地图保存: 接收`/save_map`话题触发pgm/yaml保存

**Launch文件**:
| Launch | 功能 | 修改日期 |
|--------|------|---------|
| `2d_mapper.launch.py` | lidar_tf + offline_map_publisher + mapper_node | **2026-04-30** |
| `amcl.launch.py` | map_server + AMCL + 自动初始化 + lifecycle | 2026-04-23 |
| `navigation.launch.py` | pointcloud_to_laserscan + Nav2全栈 | 2026-04-29 |

**已生成的地图版本** (说明在持续调参):
```
maps/0428_good/    — 2026-04-28 第1版(较好)
maps/0428_2/       — 2026-04-28 第2版
maps/0428_2_90/    — 2026-04-29 第3版(参数调整后)
result/2d_map/     — 离线转换结果
```

**完成度**: ★★★☆☆ 正在积极开发, mapper_node 4月30日仍在修改, 导航栈集成在调试中

---

### 2.4 livox_ros_driver2 — Livox雷达驱动

**位置**:
- `alfa_robot/src/livox_ros_driver2/` (主项目)
- `fast_lio_ws/src/livox_ros_driver2/` (SLAM空间)
- `fast_lio_ws/Livox-SDK2/` (底层SDK)

**功能**: Livox LiDAR的ROS2驱动, 支持Avia/Mid-360/HAP
- 发布话题: `/livox/lidar` (CustomMsg), `/livox/imu`
- 配置: `MID360_config.json`

**完成度**: ★★★★★ 官方驱动, 稳定无需修改

---

### 2.5 box_perception — 箱体感知

**位置**: `alfa_robot/src/box_perception/`

**功能**: 基于点云的箱体检测与分割, 与D455深度相机配合
- `perception_node.py`: 主感知节点
- `segmentor.py`: 点云分割
- `frustum_filter.py`: 视锥体过滤
- `face_fitter.py`: 面拟合
- `mask_assigner.py`: 掩码分配

**完成度**: ★★★★☆ 已集成到主项目

---

### 2.6 Faster-LIO (ROS1早期版, 已废弃)

**位置**: `testlidarV2/src/Faster-LIO/`, `testlidar_V2/src/Faster-LIO/`

**说明**: 2-3月份的ROS1(catkin)早期实验, 项目已全面迁移到ROS2的Fast-LIO2, 这些工作空间不再使用

---

## 三、整体技术架构

```
┌─────────────────────────────────────────────────────────┐
│                      硬件层                              │
│  Livox Mid-360 (4线LiDAR + 内置IMU)                      │
│  Intel RealSense D455 (深度相机, box_perception用)        │
└────────────┬────────────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────────────┐
│                      驱动层                              │
│  Livox-SDK2 ──→ livox_ros_driver2 (ROS2)                │
│  输出: /livox/lidar, /livox/imu                          │
└────────────┬────────────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────────────┐
│                    SLAM层                                │
│  Fast-LIO2 (紧耦合LiDAR-IMU Odometry & Mapping)         │
│  · IEKF (迭代扩展卡尔曼滤波)                               │
│  · ikd-Tree (增量KD树)                                   │
│  · 重力对齐 + 外参在线估计                                  │
│  输出: /cloud_registered, /Odometry, TF(camera_init→body) │
└──────┬─────────────────────┬────────────────────────────┘
       │                     │
┌──────▼──────┐    ┌────────▼─────────────────────────────┐
│  重定位层    │    │            2D地图层 (lio_2d_mapper)    │
│  ICP全局匹配 │    │  · mapper_node: 3D→2D栅格(在线)       │
│  Open3D ICP  │    │  · lidar_tf_node: 3D→2D TF投影       │
│  多尺度匹配  │    │  · pcd_to_grid_map: PCD→2D(离线)      │
└─────────────┘    │  · offline_map_publisher: 地图回放     │
                   │  · lidar_init_pose_node: AMCL初始化    │
                   └──────┬────────────────────────────────┘
                          │
┌─────────────────────────▼────────────────────────────────┐
│                    导航层                                 │
│  pointcloud_to_laserscan (3D→2D LaserScan)               │
│  Nav2: AMCL(定位) + Planner(规划) + Controller(控制)      │
│  输出: /cmd_vel_nav                                      │
└─────────────────────────┬────────────────────────────────┘
                          │
┌─────────────────────────▼────────────────────────────────┐
│                    感知层                                 │
│  box_perception: 点云分割 + 视锥体过滤 + 面拟合            │
│  fov_filter_node: 雷达点云按D455 FOV裁剪                   │
└─────────────────────────────────────────────────────────┘
```

---

## 四、开发时间线

```
2026-02  │ Faster-LIO (ROS1) 早期实验
         │
2026-03  │ 迁移到ROS2, Fast-LIO2初始集成, testlidarV2最后使用
         │
2026-04-18 │ lio_2d_mapper开始开发 (mapper_node.hpp)
2026-04-23 │ AMCL自动初始化完成(lidar_init_pose_node), amcl.launch.py
2026-04-24 │ pcd2pgm工具集成
2026-04-25 │ FAST_LIO_LOCALIZATION2代码拉取编译
2026-04-27 │ lidar_tf_node完成, offline_map_publisher完成, 第1版2D地图生成
2026-04-28 │ 多版地图生成(0428_good, 0428_2), 全局重定位launch更新
             │ alfa_robot主项目打包(含fast_lio + livox_ros_driver2 + box_perception)
2026-04-29 │ pcd_to_grid_map_node更新, navigation.launch.py完成, 第3版地图
2026-04-30 │ mapper_node.cpp更新, 2d_mapper.launch.py更新, nav2_params.yaml配置
             │ ← 同事最后工作日
2026-05-01+│ (开发暂停)
```

---

## 五、完成度评估

### 已成熟完成, 可以直接使用

| 模块 | 说明 |
|------|------|
| **Fast-LIO2 3D建图** | 核心算法完成, Mid-360配置调优, 可跑建图保存PCD |
| **Livox Mid-360驱动** | 官方驱动, 稳定可用 |
| **PCD离线转2D地图** | `pcd_to_grid_map_node`完成, 可生成pgm/yaml供Nav2使用 |
| **全局重定位** | ICP匹配逻辑完成, 可基于预建PCD地图重定位 |
| **box_perception箱体感知** | 已集成到主项目 |
| **FOV过滤器** | 雷达点云按D455相机FOV裁剪, 已完成 |

### 正在开发中 (4月27-30日有修改)

| 模块 | 当前状态 | 阻塞点 |
|------|---------|--------|
| **lio_2d_mapper在线2D建图** | mapper_node 4月30日仍在修改, 算法迭代中 | 射线追踪参数/衰减因子调优 |
| **Nav2导航栈集成** | navigation.launch.py已完成, nav2_params配置4月30日更新 | 端到端联调可能未完全通过 |
| **AMCL自动初始化** | 代码完成(4月23日) | 与Nav2的lifecycle协调可能需验证 |
| **地图参数调优** | 3个地图版本说明在持续调参 | 2D地图质量可能还不稳定 |

### 尚未完成/需要推进

| 任务 | 优先级 | 说明 |
|------|--------|------|
| **lio_2d_mapper迁移到alfa_robot** | 高 | 主项目缺少2D建图和导航能力 |
| **FAST_LIO_LOCALIZATION2迁移到alfa_robot** | 高 | 主项目缺少重定位能力 |
| **SLAM→机械臂的完整管线** | 高 | Nav2的`/cmd_vel_nav`如何与MoveIt协调未明确 |
| **多传感器标定** | 中 | 雷达与D455相机的外参标定 |
| **自动建图→自动导航的流水线** | 中 | 一键从建图到导航的脚本 |

---

## 六、关键文件速查

```
# SLAM主开发空间
/home/ar/fast_lio_ws/
├── src/
│   ├── FAST_LIO_LOCALIZATION2/     # 3D SLAM + 重定位
│   │   ├── launch/localization.launch.py        (全局定位启动)
│   │   ├── fast_lio_localization/global_localization.py  (ICP匹配核心)
│   │   └── config/mid360.yaml                  (Mid-360配置)
│   ├── lio_2d_mapper/              # 2D建图 + 导航桥接 [开发重点]
│   │   ├── src/mapper_node.cpp                 (在线2D建图, 4/30修改)
│   │   ├── src/lidar_tf_node.cpp               (3D→2D TF投影)
│   │   ├── src/lidar_init_pose_node.cpp        (AMCL初始化)
│   │   ├── src/pcd_to_grid_map_node.cpp        (离线PCD→2D)
│   │   ├── src/offline_map_publisher.cpp       (离线地图发布)
│   │   ├── launch/2d_mapper.launch.py          (2D建图启动, 4/30修改)
│   │   ├── launch/amcl.launch.py               (AMCL定位启动)
│   │   ├── launch/navigation.launch.py         (Nav2导航启动)
│   │   ├── config/nav2_params.yaml             (Nav2参数, 4/30修改)
│   │   └── maps/                               (已生成的多版2D地图)
│   ├── livox_ros_driver2/          # Livox驱动
│   └── box_perception/             # 箱体感知(旧版)
├── bags/                           # 录制的bag文件
├── map_2d.pgm / map_2d.yaml        # 根目录2D地图
└── Livox-SDK2/                     # Livox底层SDK

# 主项目仓库
/home/ar/lhy_dev/alfa_robot/src/
├── fast_lio/                       # Fast-LIO2 (已集成, 含fov_filter)
├── livox_ros_driver2/              # Livox驱动 (已集成)
├── box_perception/                  # 箱体感知 (已集成)
├── box_perception_msgs/             # 感知消息定义
├── alfa_robot_bringup/             # 启动配置
├── alfa_robot_description/         # 机器人URDF描述
├── alfa_robot_hardware/            # 硬件驱动
└── alfa_robot_moveit_config/       # MoveIt配置
```

---

## 七、推荐推进路线

### 第一步: 跑通fast_lio_ws中的完整管线
```bash
# 终端1: 启动雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2: 启动Fast-LIO2建图
ros2 launch fast_lio mapping.launch.py

# 终端3: 启动2D建图
ros2 launch lio_2d_mapper 2d_mapper.launch.py

# 建图完成后保存:
ros2 topic pub /save_map std_msgs/msg/Empty --once
```

### 第二步: 验证AMCL定位+Nav2导航
```bash
# 终端1: 雷达驱动
# 终端2: Fast-LIO2 (同上)
# 终端3: 2D建图
# 终端4: AMCL定位
ros2 launch lio_2d_mapper amcl.launch.py
# 终端5: Nav2导航
ros2 launch lio_2d_mapper navigation.launch.py
```

### 第三步: 将关键模块迁移到alfa_robot主项目
需要迁移的包:
- `lio_2d_mapper/` → `alfa_robot/src/lio_2d_mapper/`
- `FAST_LIO_LOCALIZATION2/` → `alfa_robot/src/fast_lio_localization/`
- 更新 `dependency.repos` 和启动脚本

### 第四步: SLAM→MoveIt集成
- Nav2的`/cmd_vel_nav`与机械臂控制的协调
- 定位结果(`/Odometry`或AMCL的`/amcl_pose`)作为MoveIt的机器人状态输入
