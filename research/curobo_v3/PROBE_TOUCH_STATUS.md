# 测试杆同点多朝向打表：当前测试版

**接手入口（2026-10-20整理）：[PROBE_TOUCH_HANDOFF.md](PROBE_TOUCH_HANDOFF.md)。** 本页下方保留开发历史；恢复部署、操作和当前边界请优先读交接文档。

## 2026-10-19 当前交付：8162可交互测试

历史“开发暂停”已解除，以下旧条目保留用于追溯。入口http://127.0.0.1:8162/，systemd用户服务alfa-probe-touch-8162。支持拖动XYZ/旋转环或输入XYZ/RPY（footprint坐标），实时反馈对齐临时杆尖；固定-.3m升降、左7轴，原朝向与局部XYZ各±25度（可调0～30度），3度ψ枚举、分支保留、限位与FK复核。按朝向分组和关节距离选择，不自动运动。

所选解从新读取的实测左臂姿态做关节空间直线、五次时间律插值（不是TCP直线，不绕障）；默认5度/s，可选10/15，10Hz、≤60秒、≤1000点，插值内部v/a/jerk及实机软限位复核。完整播放预览并勾确认后，用户获取18121控制权，再明确点击发送FJT。实际起点偏离预览起点>.05度会拒绝，不硬接；目标/解/速度变化清除预览。只取消本后端Action，不是硬件急停。

用户明确要求参考碰撞仅提示、不作为发送门禁：报告保留collision_passed/first_collision_s/collision_frames，不把失败说成通过；这一策略仅允许task=probe_touch并显式collision_advisory=true，原前缀门禁不变。新实时起点仍可能触发参考碰撞，模型右臂为Home且无实测环境、工作台、测试杆碰撞，因此此工具不宣称完整实机安全。上次底盘重复球对已修正，不等于任何后续姿态均安全。

部署：IPC原目录/home/ar/alfa_left_prefix_control_20261018的server.py/smooth_timing.py已更新，旧server备份server.py.before_probe_touch_20261019；原18121未改。18122→本机8155转发复用，不新增控制器。不自动使能/获取控制权。本轮真实/api/prepare接受，motion_publications=0；仅mock publisher和准备接口验证，未做实机运动验收。20项CPU回归通过，真实页面求解约4128解、切换X+25度候选并生成11.8秒轨迹；数量随姿态不同。

保存：generated/probe_touch_runs/latest_ik.json及preview_<哈希前16位>.json；算法、TCP定义、fixed Updown及碰撞提示在源代码/预览中保留。8152/8156/8160旧入口未替换。不要同时用8155旧前缀按钮与8162打表发送；先在原控制台停稳、释放控制权，再由用户操作8162。

启动命令（服务已运行，不要重复占端口）：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3 && /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/probe_touch_app.py --port 8162 --backend http://127.0.0.1:8155
```

## 更正：底盘重复检测已定位并修复

用户指出权威底盘早已使用cuboid。ProbeCollision直接加载fixed.yaml后遗漏了正式BallFullCyclePlanner中的replace_arm_base_pairs，导致保留旧base_link球与手臂自碰撞，同时又启用BaseCuboidLayer。已复用正式替代逻辑：仅由cuboid层负责手臂与底盘，其他自碰撞不变。重新测零位/Home：self cost均0、重叠球对均0、完整GpuValidity均True。因此下文原始阻塞不是实体碰撞证据，现已解除，GUI/执行尚未完成。修正证据/tmp/probe_collision_pairs_corrected.json，旧球诊断保留/tmp/probe_collision_pairs_legacy_base_spheres.json；8161已更新显示修正后结果。

分支：feature/probe-analytic-touch。用户确认固定Updown=-0.3m；TCP相对left_link7为[0,0,0.070]m，姿态不偏转；拖拽位置/朝向，生成原朝向及同一位置局部XYZ轴±25°候选，枚举解析冗余角和分支，供用户选择。之后从实时左7轴起点生成关节插值轨迹，先预览、后明确点击发送既定FJT。不是末端笛卡尔直线运动。两个功能分别是IK候选和插值运动。

已完成：probe_touch_ik.py固定升降/临时TCP转换至原tool0解析器，3度冗余角采样及当前冗余角、去重、限位、URDF FK复核；nearby_targets保持TCP位置、朝向偏转25度。probe_touch_motion.py五次时间律生成10Hz轨迹，精确核验RT插值速度≤15度/s（默认10）、加速度≤30度/s²、jerk≤150度/s³，最长60秒；无关节±2π偷换、无自动RRT绕行。

新碰撞门禁集成测试未通过，遵循用户遇非预期问题先停的要求暂停，尚未创建可执行前端、未部署新IPC接口、未获取控制权/使能/发送动作。原8155/8156/8160保持各自职责。

失败证据：/tmp/probe_touch_collision_blocker.json。独立初始化当前fixed.yaml碰撞检查器，固定右臂为配置Home参考、升降-.3，左臂零位及Home的self collision cost分别为0.0028553/0.0073689；world、bounds、base成本均0。该成本不是穿透深度，不能直接按米解释。尚未定位具体球对或证明实体碰撞，不能说机器人真实Home不可用，也不能绕过门禁。ground_min包含被豁免的base_link球，不据此判穿地。

后续先核对碰撞球对/当前球模型与权威配置、参考右臂状态、检查器自碰撞语义，待结论明确再接GUI和发送门禁。外部接触目标/工作台及实测右臂状态尚未建模，不能声称本轮门禁覆盖真实完整环境。保持原URDF和旧吸盘tool0未改，仅临时TCP转换。
