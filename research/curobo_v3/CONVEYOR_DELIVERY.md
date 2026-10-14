# 当前箱墙到龙头车研究场景

2026-10-14交付。算法在`alfa_robot/alfa_v3_curobo`；显示包在`robot_motion_control/main`，确切依赖固定于`config/visualization_dependency.json`。核心、URDF/球模型、STL和解析atlas在算法仓库；全部外观GLB在前端安装包。前端安装后，不需要原本机3DAssets、alfa_robot/data/visualization或其他工程师的研究目录。

## 当前冻结行为

1. 第一Home出发，Updown每5cm/ψ每10°解析接触遍历，按Home距离保留64候选。最低目标所在排及以下按现有任务填充规则构建实际碰撞环境，不把完整475箱外观模板混入规划。
2. 到3cm预接触，最后3cm沿用既有目标碰撞豁免；吸附后固定Updown/ψ解析后抽35cm。抽离途中延后负载碰撞的既有研究策略不变。
3. 多抽离根＋多目标Connect到搬运构型。side使用APPROVED_PLACEMENT_MODEL.md的新目标；top保留原目标。
4. 根据固定箱心反算停稳底盘位，用base_reposition一帧表示等待导航到位；不生成或验证底盘运动路径。龙头车world中心x=-2.30m、y=±0.495m，宽0.94m，间隙0.05m，STL2.4×0.94×0.732m。
5. 龙头车头局部x=1.20m，光圈半径0.35m，中心x=0.85m，向内横移0.07m；车头侧光圈边缘与端面对齐。固定箱心(-1.45,±0.425,1.182)m，箱高0.4m，现阶段箱底高于台面0.25m。这是用户抬高后的释放演示，不是物理落稳仿真。
6. 底盘冻结，cuRobo数值IK求固定最终双TCP目标候选，多目标RRT连接并细验。0.8m定义是到位后相对搬运箱心的水平位移，只用于反算停车位与最终目标，不要求箱子直线路径。
7. 释放后附着箱从碰撞和显示消失。取消5cm直线退离，直接RRT回同一第一Home，然后base_return瞬移回抓取位。

所有机械臂规划保持上下身15维（Updown＋左右七轴）。最终放箱和回Home搜索每段2秒。含机器人/双负载/集装箱/双龙头车STL与4内部底盘cuboid检查；导航瞬移不等于执行路径碰撞验收。模型为研究analytic proxy，40负载内接球、接触豁免等不代表精确STL或实机安全。

## 安装和启动

需要原cuRobo v2/main兼容环境、CUDA及现有解析桥编译依赖，版本见RESEARCH_SYNC.md和CUROBO_MAIN_COMPAT.md。在本机已有venv直接使用：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m pip install --no-build-isolation --no-deps \
  'alfa-motion-viser @ git+ssh://git@192.168.100.141:2222/SevenovaHangzhou/robot_motion_control.git@479ce2a12db558816d8d7e88434f55dbb1904005#subdirectory=observability/viser'
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/prepare_checkout.py --compiler g++
PYTHONDONTWRITEBYTECODE=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
  research/curobo_v3/tools/v3_visualized_task_demo.py \
  --port 8150
```

访问 http://127.0.0.1:8150/ ，选择任务计算、播放或按阶段查看。换checkout路径后必须重跑prepare_checkout.py重定位模型网格路径；它只在现有checkout生成解析库与路径，不另复制仓库。其他机器请使用该机器对应python/工作目录；Git需要有上述公司仓库访问权限。

前端启动验证安装包的渲染器/资产清单hash。`--frontend-root`用于已有前端checkout，`--asset-root`/`--floor-asset`/`--marker-asset`为显式自定义外观覆盖；默认使用已安装包的冻结资源。

## 回放和验收

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
PYTHONDONTWRITEBYTECODE=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
  research/curobo_v3/tools/v3_conveyor_regression.py \
  --output-dir /tmp/conveyor_delivery_validation
```

按顺序验证side L18/R16与top L4/R2；任一失败立即停，不扩预算或换策略。完整结果保存于指定输出目录。审计检查具名帧、0.5°最大相邻旋转步长、固定底盘、release→Home→导航返回、箱体移除及独立移动URDF FK≤2mm/1°。运行CPU合同测试：`python -m unittest discover -s research/curobo_v3/tests -p 'test_core_*.py'`。

只读预载已保存结果：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
PYTHONDONTWRITEBYTECODE=1 /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
  research/curobo_v3/tools/v3_visualized_task_demo.py \
  --port 8150 \
  --result /tmp/conveyor_delivery_validation/L18_R16.json /tmp/conveyor_delivery_validation/L4_R2.json
```

包内不提交全部轨迹和运行缓存，代表验证摘要见evidence/conveyor_delivery_validation.json。旧失败连接只用于诊断：勾选“显示被拒绝的碰撞连接”并点击“定位首个碰撞帧”，紅球及紅障碍定位实际被拒绝状态，不能执行。当前最终放箱路径不使用逐点直线IK，该诊断方法保留供历史结果查看。

## 显示边界

- 完整集装箱框架用统一装配变换匹配规划内壁边界；外观细节不自动成为碰撞几何。剖视只隐藏墙板，切换碰撞视图显示实际Snapshot盒/网格。
- 木地板、49彩色网格、双龙头车和光圈由前端包提供；机器人关节/工具来自当前算法URDF。GUI不反写真实状态、不做穿地回滚。
- 单渲染队列更新yourdfpy/Viser。播放帧间隔由最大相邻关节步长÷30°/s统一换算，Updown同比例，底盘瞬移不参与角速度统计；不是加速度或动力学时间参数化。
- 本次只交付当前研究基线，不宣称94任务全通过，不接ROS Action/真机/导航，不发布新硬件description。
