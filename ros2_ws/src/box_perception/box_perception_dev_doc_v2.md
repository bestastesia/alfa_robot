# 物流箱拆垛机器人视觉感知系统
## 开发设计文档 v2.0

> ROS 2 · RGB Camera (D455) · LiDAR · YOLO Segmentation · Open3D

---

## 目录

1. [系统概述与目标定义](#1-系统概述与目标定义)
2. [环境与依赖版本](#2-环境与依赖版本)
3. [项目结构](#3-项目结构)
4. [ROS 2 消息定义](#4-ros-2-消息定义)
5. [模块文档与算法接口](#5-模块文档与算法接口)
6. [标定流程（开发前置）](#6-标定流程开发前置)
7. [测试计划](#7-测试计划)
8. [开发顺序与里程碑](#8-开发顺序与里程碑)
9. [风险登记与常见问题](#9-风险登记与常见问题)
10. [附录：配置文件模板](#10-附录配置文件模板)

---

## 1 系统概述与目标定义

### 1.1 任务目标

本系统为物流箱拆垛机器人提供视觉感知能力，完成从原始传感器数据到结构化箱体位姿信息的全链路处理。

| | 项目 | 说明 |
|---|---|---|
| **输入** | RGB 图像帧 | Intel RealSense D455，分辨率最高 1280×720 |
| **输入** | LiDAR 点云 | 外置雷达（下倾 15° 安装），PointCloud2 格式 |
| **输出** | 逐箱位姿消息 | 每帧对每个检测到的纸箱发布：两个竖直面法向量 + 最近面重心 (X, Y, Z)，**雷达坐标系**下 |

### 1.2 核心约束与简化假设

- 纸箱为长方体，各面平行或垂直于地面。
- 每个可见箱子最多暴露 2 个竖直面，输出这 2 个面的法向量即可完整描述箱体 yaw 位姿。
- 深度与位姿均在**雷达坐标系**下处理和输出；雷达坐标系与机械臂坐标系一致，无需额外变换。
- RGB 图像仅用于 YOLO 语义分割，获取 instance mask；深度与几何信息全部来自 LiDAR。
- 每帧独立处理，无跨帧追踪需求（后续可扩展）。
- 雷达与相机已物理固连，外参在离线标定阶段一次性确定。

### 1.3 整体数据流

```
ROS Bag / 在线传感器
  ├── /camera/image_raw     (RGB)
  └── /lidar/points         (PointCloud2)
         │
         ▼
┌─────────────────────────────────────────┐
│  ① 时间同步 ApproximateTimeSynchronizer  │
└──────────────────┬──────────────────────┘
                   │ (rgb, cloud) 对齐帧
          ┌────────▼────────┐
          │ ② FOV 视锥过滤  │  在雷达坐标系下，用相机 FOV 对应的 6 个半空间
          │                 │  约束一次性向量化裁剪点云，丢弃视野外点
          └────────┬────────┘
                   │ 过滤后点云 (M << N)
          ┌────────▼────────┐
          │ ③ YOLO 分割     │  输入 RGB → List[InstanceResult]（mask + bbox + box_id）
          └────────┬────────┘
                   │
          ┌────────▼────────┐
          │ ④ mask→点云分配 │  将 mask 腐蚀后构建 label_image；
          │                 │  对每个点投影到像素坐标，查 label_image 获得 box_id；
          │                 │  结果：Dict[box_id → 点云子集]，全程在雷达坐标系下
          └────────┬────────┘
                   │
          ┌────────▼────────┐
          │ ⑤ 并行面拟合    │  ThreadPoolExecutor，每个 box_id 一个任务：
          │                 │  法向量估计 → 法向量 K-means(k=2) → 逐 cluster RANSAC
          │                 │  → 过滤顶面 → 排序取最近面面心
          └────────┬────────┘
                   │
          ┌────────▼────────┐
          │ ⑥ 打包发布      │  → /box_perception/result（雷达坐标系）
          └─────────────────┘
```

---

## 2 环境与依赖版本

### 2.1 运行环境一览

| 组件 | 版本要求 | 备注 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 LTS | 与 ROS 2 Humble 官方适配 |
| ROS 2 | **Humble Hawksbill** | LTS，2027 年 EOL |
| Python | 3.10.x | Ubuntu 22.04 默认版本 |
| CUDA | 11.8 / 12.x | 用于 YOLO 推理 |
| cuDNN | ≥ 8.6 | 配合 CUDA 版本 |
| PyTorch | ≥ 2.0 | `pip install torch torchvision` |
| Ultralytics YOLO | ≥ 8.0 | `pip install ultralytics` |
| Open3D | 0.17.x | 法向量估计 / RANSAC 平面拟合（C++ 底层，释放 GIL，支持真并行） |
| OpenCV | 4.x | `pip install opencv-python`，用于 mask 腐蚀与 label_image 构建 |
| NumPy | ≥ 1.24 | 向量化视锥过滤、投影 |
| SciPy | ≥ 1.10 | 法向量 K-means 聚类（`scipy.cluster.vq`） |
| ros2_numpy | Humble 分支 | PointCloud2 ↔ ndarray 转换 |
| message_filters | 随 ROS 2 安装 | 时间同步 |
| cv_bridge | 随 ROS 2 安装 | Image ↔ OpenCV 转换 |
| Intel RealSense SDK | 2.x (librealsense2) | 可选，仅在线模式需要 |

### 2.2 ROS 2 Package 依赖

```bash
# 系统包（apt）
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-py \
  ros-humble-pcl-ros

# Python 包（pip）
pip install ultralytics open3d opencv-python numpy scipy
pip install ros2-numpy
```

### 2.3 环境验证命令

```bash
ros2 --version
python -c "from ultralytics import YOLO; print('YOLO OK')"
python -c "import open3d as o3d; print(o3d.__version__)"
ros2 bag info <your_bag_path>
ros2 bag play <your_bag_path> --topics /camera/image_raw /lidar/points
```

---

## 3 项目结构

```
box_perception/
├── package.xml
├── setup.py
├── setup.cfg
│
├── msg/
│   ├── BoxResult.msg
│   └── BoxPerceptionResult.msg
│
├── config/
│   └── calib.yaml                  # 外参标定结果（离线生成）
│
├── launch/
│   ├── perception.launch.py
│   └── calibration_check.launch.py
│
├── box_perception/
│   ├── __init__.py
│   ├── perception_node.py          # 主节点入口
│   ├── frustum_filter.py           # ② 视锥过滤（雷达系半空间约束）
│   ├── segmentor.py                # ③ YOLO 封装
│   ├── mask_assigner.py            # ④ label_image 构建 + 点云分配
│   ├── face_fitter.py              # ⑤ 法向量估计 + K-means + RANSAC
│   ├── calib_loader.py             # 标定参数加载
│   └── utils.py
│
└── test/
    ├── data/
    │   ├── sample_rgb.png
    │   ├── sample_cloud.npy
    │   └── sample_mask.npy
    ├── test_frustum_filter.py
    ├── test_mask_assigner.py
    └── test_face_fitter.py
```

---

## 4 ROS 2 消息定义

### 4.1 BoxResult.msg

```
# 单个箱子的感知结果（所有坐标均在雷达坐标系下）
std_msgs/Header header
int32           box_id              # 当前帧内编号，从 0 开始（来自 YOLO instance id）
float32         confidence          # YOLO 置信度 [0, 1]

# 最近面几何（雷达坐标系）
geometry_msgs/Point   nearest_face_center   # 最近面重心，单位 m
geometry_msgs/Vector3 nearest_face_normal   # 最近面法向量，单位向量，指向雷达

# 两个竖直面法向量（雷达坐标系）
# face_normals[0]: 内点更多的面（置信度更高）
# face_normals[1]: 内点较少的面（若只检测到 1 个面则置零）
geometry_msgs/Vector3[2] face_normals

# 辅助信息
int32[2] face_inlier_counts         # 每个面 RANSAC 内点数
sensor_msgs/RegionOfInterest bbox   # 图像中的 bounding box
```

### 4.2 BoxPerceptionResult.msg

```
std_msgs/Header  header
int32            frame_id
BoxResult[]      boxes
```

### 4.3 发布话题

| 话题名 | 消息类型 | 说明 |
|---|---|---|
| `/box_perception/result` | `BoxPerceptionResult` | 主输出，每帧发布一次 |
| `/box_perception/debug/image` | `sensor_msgs/Image` | YOLO mask + 投影点可视化（调试用） |
| `/box_perception/debug/markers` | `visualization_msgs/MarkerArray` | RViz2 法向量箭头（雷达系） |

---

## 5 模块文档与算法接口

### 5.1 CalibLoader — 标定参数加载

从 `config/calib.yaml` 读取外参与内参，启动时加载一次，全局复用。

```python
class CalibLoader:
    def __init__(self, calib_path: str):
        """
        calib.yaml 格式：
          T_lidar2cam: [row-major 4x4]   # 雷达→相机坐标系变换
          camera_matrix: [row-major 3x3] # K
          dist_coeffs: [k1,k2,p1,p2,k3]
          image_width, image_height
          z_min_m, z_max_m
        """

    @property
    def T_lidar2cam(self) -> np.ndarray:   # (4, 4)

    @property
    def R_lidar2cam(self) -> np.ndarray:   # (3, 3) 旋转子矩阵

    @property
    def t_lidar2cam(self) -> np.ndarray:   # (3,)  平移

    @property
    def K(self) -> np.ndarray:             # (3, 3)

    @property
    def D(self) -> np.ndarray:             # (5,)

    @property
    def img_shape(self) -> tuple:          # (H, W)
```

> ⚠️ `T_lidar2cam` 已包含雷达 15° 下倾及相机位置偏移，标定时一并标出，不得事后手动叠加旋转。

---

### 5.2 FrustumFilter — 视锥过滤（雷达坐标系）

**原理**：相机视野等价于 6 个半空间（左/右/上/下/近/远平面）的交集。将这 6 个平面的法向量从相机坐标系变换到雷达坐标系后，过滤仅需一次矩阵乘法，全程向量化，无循环。

**预计算（节点启动时执行一次）**：

```python
class FrustumFilter:
    def __init__(self, calib: CalibLoader):
        """
        从内参 K、图像尺寸和深度范围，在相机系构造 6 个 frustum 平面法向量，
        再用 R_cam2lidar（= R_lidar2cam.T）变换到雷达系，存储备用。
        """
        fx, fy = calib.K[0,0], calib.K[1,1]
        cx, cy = calib.K[0,2], calib.K[1,2]
        W, H   = calib.img_shape[1], calib.img_shape[0]

        # 4 个侧面法向量（相机系，指向视锥内侧）
        # 形式：n · p_cam >= 0 为"在视锥内"
        left   = np.array([ fx,  0,  cx      ])   # u >= 0
        right  = np.array([-fx,  0,  W - cx  ])   # u <= W
        top    = np.array([  0, fy,  cy      ])   # v >= 0
        bottom = np.array([  0,-fy,  H - cy  ])   # v <= H
        # 2 个深度平面
        near   = np.array([0, 0,  1]) * calib.z_min_m  # 单独处理
        far    = np.array([0, 0, -1]) * calib.z_max_m

        normals_cam = np.stack([left, right, top, bottom])  # (4, 3)
        R_c2l = calib.R_lidar2cam.T
        self._normals_lidar = (R_c2l @ normals_cam.T).T     # (4, 3)，预计算完毕

        # 深度约束转化为雷达系线性不等式
        # z_cam = R_lidar2cam[2,:] · p_lidar + t_lidar2cam[2]
        self._r2 = calib.R_lidar2cam[2, :]   # (3,)
        self._t2 = calib.t_lidar2cam[2]
        self._z_min = calib.z_min_m
        self._z_max = calib.z_max_m

    def filter(self, points: np.ndarray) -> np.ndarray:
        """
        输入: (N, 3) 雷达坐标系点
        输出: (M, 3) 过滤后点，M <= N

        实现：
          侧面约束: (points @ normals_lidar.T) >= 0，shape (N, 4)，全部满足则保留
          深度约束: z_min <= points @ r2 + t2 <= z_max
          取逻辑与，一次 boolean indexing 完成
        """
        side_ok  = (points @ self._normals_lidar.T >= 0).all(axis=1)
        z_cam    = points @ self._r2 + self._t2
        depth_ok = (z_cam >= self._z_min) & (z_cam <= self._z_max)
        return points[side_ok & depth_ok]
```

---

### 5.3 Segmentor — YOLO 语义分割

```python
@dataclass
class InstanceResult:
    box_id:     int
    mask:       np.ndarray    # bool, shape (H, W)，腐蚀前
    bbox:       tuple         # (x1, y1, x2, y2) 像素坐标
    confidence: float


class Segmentor:
    def __init__(
        self,
        model_path:     str,
        device:         str   = 'cuda',
        conf_threshold: float = 0.5,
    ):

    def infer(self, rgb: np.ndarray) -> List[InstanceResult]:
        """
        输入: BGR uint8 图像 (H, W, 3)
        输出: 检测到的所有纸箱实例列表，box_id 即 YOLO instance 顺序编号
        目标耗时: < 30ms / frame（RTX 3060）
        """
```

---

### 5.4 MaskAssigner — label_image 构建与点云分配

**核心思路**：将 YOLO 的 instance mask 映射到点云，而非将点云投影到图像上去查找——两者数学等价，但后者（构建 label_image + 向量化查表）更高效。

```python
class MaskAssigner:
    def __init__(self, calib: CalibLoader, mask_erode_px: int = 8):

    def build_label_image(
        self,
        instances: List[InstanceResult]  # 来自 Segmentor
    ) -> np.ndarray:                     # int32, shape (H, W)，0=背景
        """
        步骤：
          1. 初始化全零 label_image
          2. 按 confidence 从低到高排序 instances
          3. 对每个 instance，先用 cv2.erode(kernel=ones(k,k)) 腐蚀 mask
          4. 将腐蚀后 mask 区域写入 label_image，值为 box_id+1（0 保留给背景）
             高 confidence 后写，自然覆盖低 confidence 的重叠区域
          5. 重叠边缘在腐蚀时已被置零，无需额外处理
        """

    def assign(
        self,
        points_lidar:  np.ndarray,   # (M, 3) 已经过视锥过滤的雷达系点
        label_image:   np.ndarray,   # (H, W) int32
    ) -> Dict[int, np.ndarray]:      # box_id → (K, 3) 点云子集
        """
        步骤：
          1. 外参变换：points_cam = (R_lidar2cam @ points_lidar.T).T + t_lidar2cam
          2. 内参投影：u = fx * x/z + cx, v = fy * y/z + cy
          3. 过滤超出图像边界的点
          4. 向量化查表：box_labels = label_image[v_int, u_int]
          5. 过滤 box_labels == 0（背景）的点
          6. 按 box_id 分组，返回 Dict
        所有操作均为 numpy 向量化，无 Python 循环。
        """
```

> **注意**：投影这一步的目的仅是查 label，投影完成后，返回的点云仍为**雷达坐标系**，后续不再使用相机坐标系。

---

### 5.5 FaceFitter — 面法向量估计与最近面提取

每个箱子的点云子集独立处理，由 `ThreadPoolExecutor` 并发调用。Open3D 的 C++ 底层会释放 Python GIL，多线程可实现真并行。

```python
@dataclass
class FaceResult:
    normals:            List[np.ndarray]   # 最多 2 个竖直面法向量，shape (3,)，雷达系
    inlier_counts:      List[int]
    nearest_face_center: np.ndarray        # (3,) 最近面面心，雷达系
    nearest_face_normal: np.ndarray        # (3,) 最近面法向量


class FaceFitter:
    def __init__(
        self,
        ransac_threshold:  float = 0.015,  # 内点距离阈值，单位 m
        min_inliers:       int   = 10,
        normal_radius:     float = 0.05,   # 法向量估计邻域半径，单位 m
        vertical_tol_deg:  float = 30.0,   # 竖直面判定：法向量与水平面夹角 < 此阈值
        max_workers:       int   = 4,
    ):

    def fit_all(
        self,
        box_clouds: Dict[int, np.ndarray]  # box_id → (K, 3) 雷达系点云
    ) -> Dict[int, Optional[FaceResult]]:
        """
        对每个 box_id 提交 ThreadPoolExecutor 任务，并发执行 _fit_one，
        收集结果返回。
        """

    def _fit_one(
        self,
        points: np.ndarray   # (K, 3) 雷达系
    ) -> Optional[FaceResult]:
        """
        单箱处理流程（在线程内执行）：

        Step 1 — 法向量估计
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(self.normal_radius))
            normals = np.asarray(pcd.normals)   # (K, 3)

        Step 2 — 过滤顶面点
            竖直面条件：|normal_z| < sin(vertical_tol_deg)
            vertical_mask = |normals[:, 2]| < threshold
            若 vertical_mask.sum() < min_inliers，返回 None

        Step 3 — 法向量 K-means（k=2）
            对过滤后点的法向量做 scipy.cluster.vq.kmeans2(normals_vertical, k=2)
            获得 2 个聚类中心方向，对应箱子的 2 个竖直面方向

        Step 4 — 逐 cluster RANSAC 平面拟合
            对每个 cluster 的点调用 open3d segment_plane()
            精确化法向量和面心；过滤内点数 < min_inliers 的 cluster

        Step 5 — 最近面判定
            雷达坐标系中，z 轴为光轴方向，取所有拟合平面面心的 z 值最小者
            （若坐标系不同，改为按雷达原点到面心的距离排序）
            输出最近面面心坐标和法向量

        Step 6 — 法向量方向统一
            确保每个法向量指向雷达原点方向（normal · (-face_center) > 0），
            否则取反
        """
```

---

### 5.6 PerceptionNode — 主节点

```python
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('box_perception_node')
        self.calib    = CalibLoader(self.get_parameter('calib_path').value)
        self.frustum  = FrustumFilter(self.calib)          # 预计算，启动时一次
        self.seg      = Segmentor(...)
        self.assigner = MaskAssigner(self.calib, mask_erode_px=8)
        self.fitter   = FaceFitter(max_workers=4)

        self.sub_img = Subscriber(self, Image,       '/camera/image_raw')
        self.sub_pc  = Subscriber(self, PointCloud2, '/lidar/points')
        self.sync    = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_pc], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        self.pub_result  = self.create_publisher(BoxPerceptionResult, '/box_perception/result', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/box_perception/debug/markers', 10)
        self.pub_debug   = self.create_publisher(Image, '/box_perception/debug/image', 10)

    def callback(self, img_msg: Image, pc_msg: PointCloud2):
        rgb    = bridge.imgmsg_to_cv2(img_msg)
        cloud  = ros2_numpy.numpify(pc_msg)[:, :3]   # (N, 3)

        # ② 视锥过滤（雷达系，向量化）
        cloud_fov = self.frustum.filter(cloud)        # (M, 3)

        # ③ YOLO 分割
        instances = self.seg.infer(rgb)               # List[InstanceResult]

        # ④ 构建 label_image，点云分配（雷达系）
        label_image = self.assigner.build_label_image(instances)
        box_clouds  = self.assigner.assign(cloud_fov, label_image)  # Dict[box_id → (K,3)]

        # ⑤ 并行面拟合
        face_results = self.fitter.fit_all(box_clouds)  # Dict[box_id → FaceResult]

        # ⑥ 打包发布
        result = BoxPerceptionResult()
        result.header = img_msg.header
        for inst in instances:
            face = face_results.get(inst.box_id)
            if face is None:
                continue
            box = BoxResult()
            box.box_id                = inst.box_id
            box.confidence            = inst.confidence
            box.nearest_face_center   = to_point_msg(face.nearest_face_center)
            box.nearest_face_normal   = to_vector3_msg(face.nearest_face_normal)
            box.face_normals          = [to_vector3_msg(n) for n in face.normals]
            box.face_inlier_counts    = face.inlier_counts
            result.boxes.append(box)

        self.pub_result.publish(result)
```

---

## 6 标定流程（开发前置）

> ⚠️ **标定精度是全系统准确性的瓶颈。若外参误差 > 5px，最终 XYZ 定位误差可能超过 2cm，直接影响机械臂抓取。请务必在开始功能开发前完成标定验证。**

### 6.1 标定目标

获得雷达到相机坐标系的外参矩阵 `T_lidar2cam`（4×4 齐次矩阵），以及确认 D455 内参。

### 6.2 内参获取

D455 内参可直接从 ROS 话题读取，无需单独标定：

```bash
ros2 topic echo /camera/camera_info
# 读取 K（3x3 camera matrix）和 D（畸变系数）字段，填入 calib.yaml
```

### 6.3 外参标定步骤

1. 在采集现场静置传感器，在相机视野内放置棋盘格标定板，使雷达也能扫到标定板。
2. 录制同时包含棋盘格的 RGB 帧和点云帧的 ROS Bag。
3. 使用推荐工具 `lidar_camera_calibration` 求解外参（棋盘格角点提供像素↔三维对应关系，工具自动求解 PnP 问题）。

```bash
ros2 run lidar_camera_calibration calibration_node \
  --ros-args \
  -p camera_info_topic:=/camera/camera_info \
  -p lidar_topic:=/lidar/points
```

4. 将结果写入 `config/calib.yaml`，运行可视化检查脚本。
5. **验收标准**：点云投影到图像后，角点对齐误差 ≤ 3 像素。

> **关于雷达 15° 下倾**：不要手动叠加这个旋转。标定工具在你点角点对应关系时会自动将其算入，最终 `T_lidar2cam` 矩阵已包含该旋转。

### 6.4 快速验证脚本

```bash
python tools/calibration_check.py \
    --bag <bag_path> \
    --calib config/calib.yaml \
    --output output/calib_check.png
# 功能：读取一帧 RGB + 点云，将点云按深度着色叠加投影到图像，人工确认对齐质量
```

---

## 7 测试计划

### 7.1 测试层级概览

| 层级 | 测试对象 | 测试数据 | 通过条件 |
|---|---|---|---|
| L1 单元 | 各模块独立函数 | `test/data/` 离线样本 | 断言 + 数值误差 |
| L2 集成 | 模块间接口 | Bag 截取单帧 | 可视化人工确认 |
| L3 系统 | 全链路 ROS 节点 | 完整 ROS Bag 回放 | 话题输出 + RViz2 |
| L4 性能 | 端到端延迟 | 完整 Bag 压力测试 | < 100ms / frame |

### 7.2 L1 单元测试

#### T1.1 — FrustumFilter

- [ ] 构造已知在 FOV 内的点，确认全部通过过滤
- [ ] 构造已知在 FOV 外的点（左/右/上/下/近/远各一个），确认全部被过滤
- [ ] 验证预计算的 `_normals_lidar` 与手算结果一致（误差 < 1e-6）

#### T1.2 — MaskAssigner

- [ ] **build_label_image**：两个重叠 mask，高 confidence 覆盖低 confidence；重叠腐蚀边缘为 0
- [ ] **assign**：构造已知坐标的点，验证分配到正确 box_id；视锥外点不出现在结果中
- [ ] 投影坐标边界条件：点投影到图像边缘外时不分配（不抛异常）

#### T1.3 — FaceFitter

- [ ] **理想双平面**：构造两个互相垂直的平面点云（各加 1mm 噪声），验证输出 2 个法向量互相垂直（误差 < 2°）
- [ ] **最近面选取**：两个面中 z 较小的面心应为 `nearest_face_center`
- [ ] **含离群点鲁棒性**：混入 20% 随机离群点，RANSAC 结果仍稳定
- [ ] **点数不足返回 None**：输入 < `min_inliers` 点时返回 `None`，不抛异常
- [ ] **法向量方向**：所有输出法向量满足 `n · (-face_center) > 0`（指向雷达原点）

### 7.3 L2 集成测试

#### T2.1 — 视锥过滤可视化
将过滤后点云投影回图像，确认所有点落在图像范围内，无图像外点。

#### T2.2 — mask 分配可视化
对单帧数据运行分割 + 分配，用不同颜色渲染各 box_id 点云，叠加在 RGB 图像上，确认同一箱子的点云与 mask 空间一致。

#### T2.3 — 面拟合可视化
对单个箱子点云运行 `FaceFitter._fit_one`，在 Open3D 窗口中显示：内点（绿/蓝区分两个面）、离群点（红色）、法向量箭头，确认两个面方向合理。

### 7.4 L3 系统测试

```bash
# 终端 1
ros2 bag play <bag_path> --loop

# 终端 2
ros2 launch box_perception perception.launch.py

# 终端 3
ros2 topic echo /box_perception/result
ros2 topic hz   /box_perception/result
```

RViz2 确认 `/box_perception/debug/markers` 中法向量箭头方向合理（竖直面，朝向雷达）。

### 7.5 L4 性能测试

| 指标 | 目标 | 紧急 Fallback |
|---|---|---|
| 端到端延迟（单帧） | < 100ms | 降低图像分辨率 |
| YOLO 推理 | < 30ms | 换小模型 / TensorRT |
| 视锥过滤 + 分配 | < 5ms | 已向量化，通常无需优化 |
| 面拟合（并行） | < 30ms | 降低法向量估计邻域半径；体素下采样 |

---

## 8 开发顺序与里程碑

> 💡 **原则**：每阶段结束前必须有可视化/可验证的输出。**P1（标定验证）阻塞所有后续工作，优先完成。**

| 阶段 | 预计工时 | 任务内容 | 验收标准 |
|---|---|---|---|
| P0 | 0.5 天 | 环境搭建：依赖安装；Bag 话题确认；`calib.yaml` 模板填写 | `ros2 bag info` 可读，依赖版本验证全通过 |
| P1 | 1～2 天 | **标定验证**：获取内参；运行外参标定工具；`calibration_check.py` 确认对齐 | 角点投影误差 ≤ 3px |
| P2 | 0.5 天 | 自定义消息 + 节点框架：消息文件；`PerceptionNode` 骨架 | 节点启动无报错，话题可 `echo` |
| P3 | 0.5 天 | **FrustumFilter**：视锥预计算 + 向量化过滤；T1.1 单元测试 | L1 测试通过；过滤后点云投影可视化正常 |
| P4 | 1 天 | **MaskAssigner**：label_image 构建 + 向量化点云分配；T1.2 单元测试 | L1 测试通过；分配可视化人工确认 |
| P5 | 0.5 天 | **Segmentor 接入**：接入现有 YOLO 权重，确认 box_id 传递正确 | 单帧分割结果可视化正常 |
| P6 | 1.5 天 | **FaceFitter**：法向量估计 + K-means + RANSAC + 并行框架；T1.3 单元测试 | 理想双平面误差 < 2°；并行执行无竞争 |
| P7 | 1 天 | **全链路集成**：串联所有模块；T3 系统测试；RViz2 法向量可视化 | Bag 回放下持续输出结果，无崩溃 |
| P8 | 0.5 天 | 性能调优 + 边界情况处理；L4 性能测试 | 端到端 < 100ms |
| P9 | 0.5 天 | 文档 & 代码整理 | 交接可用 |

**总计预计：7～8 个工作日**

---

## 9 风险登记与常见问题

### 9.1 风险登记

| 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|
| 标定精度不达标（> 3px） | 中 | 点云分配错位，法向量误差大 | 重采标定数据；多帧平均 |
| LiDAR FOV 与相机差异大，边缘箱子无点云 | 中 | 边缘箱子无法输出结果 | 过滤超出 LiDAR FoV 的检测实例；视锥过滤后统计空集并跳过 |
| 点云稀疏导致法向量估计不稳定 | 中 | K-means 聚类结果不可靠 | 增大 `normal_radius`；降低 `min_inliers`；动态调整 RANSAC 阈值 |
| 两个竖直面法向量 K-means 退化为 1 个方向 | 中 | 只能检测到 1 个面 | 检测 cluster 内点数，若差距过大则只输出 1 个面，`face_normals[1]` 置零 |
| 雷达与相机时间戳不对齐（> 50ms） | 低 | 投影错位 | 调整 `slop` 参数；检查传感器时钟源 |
| ThreadPoolExecutor 线程数过多导致 CPU 争抢 | 低 | 整体延迟反而增加 | `max_workers` 设为物理核数，通常 4 即可 |

### 9.2 技术 FAQ

**Q1：为什么要在雷达坐标系里做视锥过滤，而不是先变换到相机系再过滤？**

两种方式数学等价，但在雷达系过滤的好处是：后续所有操作（分配、拟合、输出）全部在雷达系下进行，不需要来回变换。`T_lidar2cam` 只在 `MaskAssigner.assign` 里做一次投影（为了查 label_image），之后雷达系点坐标原样返回。

**Q2：法向量 K-means 用的是什么距离度量？**

使用欧氏距离对法向量方向做聚类（`scipy.cluster.vq.kmeans2`）。由于法向量是单位向量，欧氏距离和角度距离在小角度下等价。注意法向量有方向歧义（同一平面法向量可指向两侧），聚类前统一将法向量 z 分量翻转为正（或统一朝向原点）以消除歧义。

**Q3：雷达 15° 下倾需要在代码里特殊处理吗？**

不需要。`T_lidar2cam` 外参矩阵已经包含该旋转。代码中一律用矩阵变换，不手动叠加任何旋转角。

**Q4：两个竖直面法向量能否完整描述箱体位姿？**

可以。两个竖直面法向量互相垂直，知道其中一个就能通过叉积推算另一个，因此 yaw 角完全确定。加上最近面面心的 (X, Y, Z)，机械臂抓取所需的位姿信息完整。

**Q5：箱子被部分遮挡怎么处理？**

遮挡面无激光回波，`FaceFitter` 自然只拟合可见面。若遮挡导致可见面点数 < `min_inliers`，`_fit_one` 返回 `None`，该实例在输出中跳过。

---

## 10 附录：配置文件模板

### config/calib.yaml

```yaml
# 雷达→相机外参（由标定工具填写，包含 15° 下倾及位置偏移）
T_lidar2cam:
  - [ 1.0,  0.0,  0.0,  -0.03 ]   # 占位示例，须替换为实测值
  - [ 0.0,  0.966, 0.259, 0.05 ]
  - [ 0.0, -0.259, 0.966, 0.0  ]
  - [ 0.0,  0.0,   0.0,   1.0  ]

# D455 相机内参（从 /camera/camera_info 读取后填入）
camera_matrix:
  - [fx,  0,  cx]
  - [ 0, fy,  cy]
  - [ 0,  0,   1]

# 畸变系数 [k1, k2, p1, p2, k3]
dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]

# 图像尺寸
image_width:  1280
image_height:  720

# 深度范围（相机系 z 轴，单位 m）
z_min_m: 0.1
z_max_m: 4.0
```

### launch/perception.launch.py 参数说明

```python
calib_path        = 'config/calib.yaml'
model_path        = 'models/box_seg.pt'
conf_threshold    = 0.5         # YOLO 置信度阈值
mask_erode_px     = 8           # mask 腐蚀像素数
ransac_threshold  = 0.015       # RANSAC 内点距离阈值（m）
min_inliers       = 10          # 最少内点数
normal_radius     = 0.05        # 法向量估计邻域半径（m）
vertical_tol_deg  = 30.0        # 竖直面判定角度阈值
max_workers       = 4           # 并行线程数
debug_publish     = True        # 关闭可省约 5ms/frame
```

---

*文档版本：v2.0 | 基于 v1.0 重构，2025年*
