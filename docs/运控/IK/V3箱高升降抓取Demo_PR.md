# [feat][motion] 单箱抓取前按目标箱高调整升降轴

- 日期：2026-09-11；来源 `bestastesia/alfa_robot:feature/wall-box-height-alignment`，目标 `kkozia188/alfa_robot:alfa_v3_dev`。
- **依赖 PR #20，请先合并 #20。** 本分支基于其冻结提交 `27ea05e3823cdb0a474142a1354cafef8f5b80b7`；#20尚未合并时，本PR对目标分支的差异会包含其内容。本次升降增量可通过 `git diff 27ea05e HEAD` 单独审阅，不改写冻结分支。
- 待审查开发版本，不自动合并、不打正式发布tag。Issue按用户此前授权留空。

## 1. 修改内容

- 抓取前计算 `descent=max(0, shoulder_center_z-(box_center_z+shoulder_box_offset))`，使用模型 `updown` 先下降，再进行原有抓取规划/回放。
- 肩中心明确取左右肩部前三轴公共交点的中点，目标取箱体中心。复用解析IK现有几何并新增模型级读取接口，原static接口保持LegacyV304语义。
- `align_height=true`、`shoulder_box_offset=0.25m`；默认演示箱号由6改为0。关闭高度调整恢复冻结版固定高度行为。
- 目标超过模型升降限位[-1,0]m则拒绝，不静默截断；下降以至多5mm间隔检查整机边界和碰撞，保持原SRDF ACM。
- IK基坐标、RRT起态和携箱返回目标均使用下降后的状态；返回阶段保持升降高度。新增完整16轴的 `lower_to_box_height` 回放阶段和JSON诊断，RViz/Rerun同步使用同一几何结果；Rerun不再将升降米数当作弧度计算播放间隔。
- 增加高度验收脚本，原冻结版回归显式关闭高度调整；更新Markdown及离线HTML指南。不混入独立冗余IK启动修复或其他未归属改动。

## 2. 修改原因

用户要求在此前确认的旧架构单臂抓取demo上，将升降轴作为抓取前可动项，先降到箱中心上方暂定25cm，再尝试抓取较低箱体。实现严格公式与可观察的失败反馈，不自行替换高度策略。

## 3. 影响范围

- `motion/sim`：`alfa_robot_analytic_ik`、`alfa_robot_moveit_config`、`alfa_robot_rerun`。
- `PlanWallBoxDemo.srv`请求/响应定义不变；JSON新增 `height_alignment` 和可选下降阶段，依赖固定五阶段或updown恒零的消费者需要适配。
- **默认行为变化：**新距离launch默认开启升降、默认box_id=0；相同x/箱号的可达结果可能与冻结版不同。用 `align_height:=false` 兼容旧固定高度行为。旧单臂launch保持原行为。
- 无新包/依赖、不改机器人网格/碰撞豁免、不接硬件或生产runtime。请核心维护者重点审查模型肩中心接口、共享轴状态传播及回放契约；尚未确认负责人名单，不冒称已完成通知/审批。

## 4. 测试情况

- [x] 本地Release构建：上述3包通过。
- [x] 单元测试：解析IK CTest 2/2通过。
- [x] launch/服务：高度版23次真实请求、3类非法偏置拒绝通过；冻结版启动/关闭回归8项通过。
- [x] 几何仿真：0～9扫描，显式左右臂、10/14、高低切换重置、非正差不抬升、超限拒绝和接近-1m边界；独立URDF FK核对肩中心、16轴固定约束、升降插值、附着连续性、RViz最终箱位与Rerun FK。
- [x] 冻结版回归通过：含10左臂/14右臂及auto、零数值间隙反例；旧demo前置测试10项通过。
- [x] Rerun录制校验通过，261帧完整覆盖下降至携箱返回末帧。
- [ ] CI、核心review及实机测试：未完成，以上均为本机测试，不能替代远端CI或实机验收。

复现（先安装仓库依赖；用空闲localhost domain，与实机隔离）：

```bash
source tools/ros_humble_env.sh
cd ros2_ws
colcon build --packages-select alfa_robot_analytic_ik alfa_robot_moveit_config alfa_robot_rerun \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
cd ..
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=192
python3 ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_height_alignment.py \
  --artifacts /tmp/wall_height_alignment
ctest --test-dir ros2_ws/build/alfa_robot_analytic_ik --output-on-failure
```

本机完整证据：`/home/astesia/Sevenova/日志/验收_2026-09-11/wall_height_alignment/`，含 `validation_summary.json`、`build_final.log`、`final_acceptance/`、冻结回归、startup、legacy及RRD校验。不提交编译产物、大型录制或原始日志；上述本机绝对路径不是远端可访问的附件，reviewer可运行提交内脚本复验。

## 5. 风险说明

- **25cm并未覆盖整两排：**x=.30m、offset=.25m、auto下，底两排成功为0、4、5、9；1、2、3、6、7、8双臂均无预接触IK候选。尤其6号冻结版成功、新高度版失败。这只表示当前策略/限位/有限搜索下未找到路径，不是绝对不可达证明；未偷偷改偏置、抓取姿态或放宽碰撞。
- 当前肩中心初始Z≈1.342432773m，底排下降≈.892432773m、第二排≈.482432773m。基准、25cm偏置和实机升降零位/行程仍需标定确认。
- 碰撞检查为离散采样，不是连续扫掠证明；未构造自然的下降中途碰撞反例。场景无地板碰撞体，未验收真实载荷、稳定性或动力学。
- 每次请求仍重置双臂零角、updown=0和完整箱墙；不是接续上一轮的回升/放置/连续拆墙。下降通过而抓取失败时，可能仅回放合法下降前缀用于诊断，不代表搬运成功。
- 只生成几何路径并回放，不输出可直接下发实机的时间/速度/加速度轨迹。

## 6. 关联 Issue

留空：用户允许无明确对应Issue时不绑定；未编造Linear ID，不使用关闭Issue的魔法词。依赖的 #20 是PR而非Issue。
