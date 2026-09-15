# [feat][motion] 冻结旧架构距离输入单箱抓取与双臂仿真演示

- 日期：2026-09-11。
- 来源分支：`feature/wall-box-grasp`（用户允许无明确对应 Issue 时留空）。
- PR 目标：`kkozia188/alfa_robot:alfa_v3_dev`；基线 `1e6c58f`，沿用用户/mentor 确认的旧架构，不跨架构合并到 main。
- 发布来源：`bestastesia/alfa_robot:feature/wall-box-grasp`；已通过 VS Code 登录核验账号，使用 GitHub 隐私邮箱署名；Issue 按用户授权留空。
- 此次冻结是待审查源码快照，不是正式发布；审查、CI、合并完成前不打正式版本 tag。

## 1. 修改内容

- 复用旧单臂 demo 可执行程序，新增独立距离输入 launch 和 demo-local `PlanWallBoxDemo.srv`：输入 `x / box_id / arm`，返回成功状态、请求代次、选中臂、失败阶段/原因及 JSON 回放。
- 固定 5×5 箱墙、箱号 0..24；左右臂及 auto 完整规划，RViz/Rerun 展示同一几何轨迹与附着箱体。
- 五阶段：接触前 RRT → 笛卡尔靠近 → 附着 → 笛卡尔抽离 → 负重 RRT 返回；加入整机状态/边碰撞及非活动轴约束检查。
- 修复非法 arm 启动后观察器残留：launch 参数 choices + 规划节点退出联动 Shutdown。
- 修复 14 号右臂零间隙贴面时的网格数值碰撞：仅新模式默认 1µm 数值间隙，接触/附着/回放统一变换；未关闭邻箱碰撞，旧 demo 默认仍为零间隙。
- 纳入本机编译前置 Conda 路径隔离修复及配套 shell 回归；不混入独立冗余 IK 的 WAITING TF 修复。
- 技术说明：`V3距离输入单箱抓取Demo.md`；可离线打开的操作指南：`V3距离输入单箱抓取Demo操作指南.html`（均在本目录）。

## 2. 修改原因

用户及 mentor 已确认在旧架构单臂 demo 上开发，先通过给定车头到箱墙距离与箱号验证单箱抓取；同时解决关闭终端重启后一直 waiting、10 号成功而 14 号失败的问题。用户已认可当前演示并要求冻结提交。

## 3. 影响范围

- interfaces：新增 `alfa_robot_moveit_config` 包内仿真专用服务，非生产控制合同；不修改已有服务字段，不属于破坏性接口变更。
- motion_control / sim / bringup：共用的单臂 C++ demo、旧/新 launch、默认箱墙参数、服务生成及回归脚本。
- 可视化：已有 Rerun viewer 兼容新请求和固定箱号；RViz 场景与选中臂状态。
- deploy：共享 `tools/ros_humble_env.sh` 的 Conda 过滤行为变更，需检查非本机环境兼容性。
- 不接入生产 runtime，不向实机控制器发送指令，不修改 IK 数学、机器人模型或主架构。

## 4. 测试情况

下列完整构建/仿真证据来自 2026-09-11 本次已验收版本；冻结整理只改文档，不重跑或干扰桌面 domain187 演示。

- [x] 本地编译通过：Release `alfa_robot_moveit_config` 单包构建；前轮 config/rerun 构建已通过。
- [ ] 独立单元测试全量通过：本次主要提供集成回归，不声称整仓单元测试通过。
- [x] launch 测试通过：8 项，含非法 arm/x/gap、三次全新启动请求6号、SIGINT/SIGTERM/SIGHUP 退出清理。
- [x] 仿真测试通过：25 箱固定墙/远距离失败、7 类非法请求、left/right/auto、五阶段、固定非活动轴、附着连续性、RViz FK、车头标定覆盖；旧 demo 10 项回归通过。
- [x] 10 号 auto → left、14 号 auto → right，桌面实际请求均成功，各 79 帧；Rerun 收到 generation3 SUCCESS。
- [x] 严格反例：`contact_numerical_gap:=0.0` 重现 14 号右臂 `right_joint7/neighbor_box_9` 最后接触碰撞。
- [x] 冻结检查：环境隔离脚本回归、Python AST、HTML 锚点和暂存差异空白检查。
- [ ] 实机测试通过：未进行，几何轨迹禁止直接下发实机。
- [ ] 远端 CI 通过：尚未创建 PR，未运行远端 CI。

复现（已安装仓库所需 ROS Humble/MoveIt/Rerun 依赖；在仓库根目录运行）：

```bash
source tools/ros_humble_env.sh
(cd ros2_ws && colcon build --packages-up-to alfa_robot_moveit_config alfa_robot_rerun \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2)
source ros2_ws/install/setup.bash
bash tools/test_ros_humble_env.sh
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=188  # 必须先确认该 domain 空闲
artifacts=$(mktemp -d /tmp/wall-grasp-review.XXXXXX)
python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_preparation.py --artifacts "$artifacts/legacy"
python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_grasp_demo.py --artifacts "$artifacts/grasp"
python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_startup.py --artifacts "$artifacts/startup"
```

原始本机证据（不随 Git 分发，不应视为 reviewer 可访问的公共链接）：
`/home/astesia/Sevenova/日志/验收_2026-09-11/wall_grasp_arm_symmetry/`。
其中 `build.log`、`legacy.log`、`regression.log`、`startup/startup_summary.json`、`visual_summary.json`、`final_checks.txt` 可供验收对照；大体积轨迹/录制不纳入源码提交。上述脚本可重新生成集成测试证据。

## 5. 风险说明

- `success` 只证明固定初态/模型、离散碰撞检查与有限搜索预算内找到完整几何路径；失败不是物理不可达证明。
- `x=0.30m` 只是仿真样例，仍待同事实测距离与基准面合同；1µm 只是仿真数值间隙，不是吸盘/TCP 标定或实机安全间距。
- 每次请求重置完整箱墙和初态，不支持连续拆墙、后放释放、共享轴搜索、动力学或吸附反馈；旧 demo 零臂角不等于主架构 home。
- 服务同步串行、无取消；DDS 发现缓存和独立查看窗口不代表服务仍在线。初始化 FATAL 时 launch 外层可能返回0，应查节点日志。
- Reviewer 重点检查：整机碰撞/附着变换、旧 demo 兼容性、服务文档、共享环境脚本；按核心模块要求至少2人 approve，其中至少1名 core-maintainer，CI 通过且无未解决讨论后才合并。
- 新接口需 core-maintainer 审查并通知受影响的运动/可视化模块负责人；负责人及通知尚待确认，不能标记已完成。
- 仓库引用的 `/home/li/.codex/RTK.md` 在本机不存在；已遵循现有根目录 Git 规范及 `.ai_teamwork` 规则。

## 6. 关联 Issue

留空（用户于 2026-09-11 明确允许）。GitHub 仓库 open/closed 列表共返回18项、均为 PR，没有独立 Issue；本地任务记录也没有能确认对应本任务的编号。未查询到 Linear 在线列表，不冒用 MOTION-94 / MOTION-154，不填写虚构的 Refs/Closes。
提交标题拟为 `feat(motion): 冻结距离输入单箱抓取与双臂仿真演示`，按仓库要求附 `Co-Authored-By: Codex <codex@openai.com>`。
