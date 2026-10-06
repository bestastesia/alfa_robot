# cuRobo 独立状态、场景与规划核心

MOTION-276 完成 MoveIt 替代的阶段1～3。当前核心位于 `tools/curobo_core`，提供
独立场景/具名机器人状态及已有混合算法的调用入口，不需要 MoveIt、ROS 或 Viser。
算法仍是 cuRobo 数值IK/FK/球碰撞、自研批量RRT及C++解析抽离。
整合目标分支cb08548d的四规划器对比入口：默认仍为Informed RRT，已有RRT、
RRTConnect及GPU PRM选择、固定接触构型对比及搜索统计保留在独立核心中。
后续在同一核心入口增加Connect＋Informed和BIT* GPU批量变体；实验边界与页面命令见[SIX_PLANNERS.md](SIX_PLANNERS.md)。

## 数据与职责

| 模块 | 职责 |
| --- | --- |
| `scene.py` | 不可变 Pose/RobotState/SceneSnapshot，SceneStore版本及吸附/释放事务 |
| `contracts.py` | 冻结资产路径、具名PlanRequest、JSON输入合同 |
| `adapter.py` | map世界对象转换到固定基座/GPU场景、完整附着变换 |
| `cache.py` | 场景几何、基座位姿、碰撞策略、任务和箱球身份形成缓存键 |
| `backend.py` | 常驻cuRobo IK/检查器及多目标运输规划 |
| `planner.py` | 接触候选、解析35cm抽离、到位/搬运/回位与结构化结果 |
| `v3_plan_cycle.py` | 无GUI输入/输出文件调用 |
| `v3_full_cycle_demo.py` | 消费同一核心和场景快照的Viser入口 |

所有世界对象以 `map` 表达，四元数为wxyz，关节值为rad/m。`RobotState`按名称取值，
缺少规划所需关节直接报错；`stamp_ns`由调用方提供，Demo使用主机时间，尚未定义ROS时钟
或现场新鲜度准入。固定模型适配器做 `T_base^-1 * T_map_object`，mobile模型使用map对象。
场景支持长方体和冻结三角网格；网格顶点为米制局部坐标，实例位姿以map表达。实时实例到规划快照的筛选见CONVEYOR.md；点云尚未接入。

`SceneStore`每次更新产生新不可变快照，以expected_revision拒绝过期写入。双箱
`attach_many()`在一次事务中移除世界副本并添加附着物；`release()`提供world_pose时
恢复世界对象，不提供时让箱体消失。规划只返回预测吸附/释放事件，不修改调用方实际场景。
结果携带request/model/scene/state身份、scene_revision和显式policy。

GPU缓存按几何身份及负载内容失效；仅反馈时间或关节值变更不重建静态世界检查器。
同任务更新障碍位置、基座位姿、附着物或碰撞策略、箱球配置时会重建。
一个FullCyclePlanner实例一次只允许一个plan_request，忙时拒绝。需要接受结果时使用
`plan_from_store()`；如果调用方SceneStore在规划期间更新，结果success=false且
error.code=SCENE_CHANGED。异步执行期的场景监控不在本阶段。

## 核心调用

在模块目录中，可以从 `tools` 导入：

```python
from curobo_core.contracts import PlannerAssets, PlanRequest
from curobo_core.planner import FullCyclePlanner

planner = FullCyclePlanner(PlannerAssets())
request = planner.demo_request({"left": 24, "right": 20})
result = planner.plan_request(request)
```

调用方可把 `request.to_dict()`保存为JSON，再用 `PlanRequest.from_dict()`读入。输入含
snapshot（模型ID、采样状态、对象、附着物、frame/version/policy）、左右任务箱ID以及
左右Tool0完整目标Pose。箱号只是本研究任务对象索引，不是新增跨域接口字段。
生产者提供的快照model_id必须与planner配置的内容身份相同。

当前完整周期支持底盘在map原点、头部两轴固定零位、一或两个不同的轴对齐标准箱；
单臂仍给空臂完整搬运目标并保留整机碰撞检查。不支持已携箱启动或任意基座位姿全周期。状态/场景层可以表达这些状态，
当前周期入口会明确拒绝不支持的输入。起点可由snapshot给定，结束时回到同一个关节状态。

## 可复制命令

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/prepare_checkout.py --compiler g++
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_plan_cycle.py \
  --left-box 24 \
  --right-box 20 \
  --runs 2 \
  --output /tmp/v3_core_cycle.json
```

输出为request和results；results保存未定时的关节帧、阶段、负载显隐、预测场景事件及
失败信息。它不是可直接交给硬件的轨迹Action，未生成速度/加速度/执行时间。
`--request /absolute/request.json`可替代Demo请求，资产路径可用CLI参数显式指定。

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python -m unittest discover \
  -s tests \
  -p 'test_core_*.py' \
  -v
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_core_smoke.py \
  --output /tmp/v3_core_gpu_smoke.json
```

CPU测试不需要cuRobo、ROS或CUDA，只需要NumPy/SciPy；GitHub CI运行CPU合同测试。
GPU专项检查还需README所述固定环境与已编译解析桥。

## 实验规则与完成边界

`CollisionPolicy.exclude_task_objects_before_contact=true`保留目标箱到位前豁免；
完整快照/显示仍有这些箱子。`defer_payload_until_extract_end=true`保留抽离途中
延后附着球碰撞、35cm终点启用的规则。地面由ground_z平面检查，正常base_link支撑
豁免；如果有ground对象，其顶面必须与该平面一致。89°箱体倾斜门限及2秒Anytime不变。

40球仍为内接近似，本次不增加真实OBB/FCL门禁，不改变抓取策略，不接RT-Control或
ROS Action，不做时间参数化，也不删除历史MoveIt包。现有规划核心自身已独立；整个
仓库彻底移除MoveIt及正式执行链属于后续阶段。

本阶段基线及迁移结果摘要见 `generated/core_migration_276/validation.json`。完整周期
成功只证明首组及现有实验规则下的几何规划，未重验11/60组或实机。

## MOTION-233整墙顺序增量（2026-10-06）

`FullCyclePlanner.plan_sequence(snapshot)`按布局生成所有有箱行：外侧双臂、内侧双臂、奇数列单臂收尾。成功后从释放事件和实际末帧生成下一轮快照；失败停止，不提交失败轮场景变化。5×5默认15轮，0.75m本轮连续实跑前14轮24箱通过，第15轮L02接触IK失败，**不是25/25验收**。

现有headless与Viser入口增加`--sequence`；Viser可通过`--sequence-json`显式回放匹配场景的结果并查看各轮。接触候选扩展到128并去重；空载回程可显式反向复用已验证去程，重新密检并记录fallback，不隐藏2秒搜索失败。原冻结模型/碰撞政策不改。完整说明见`../../docs/motion233_curobo_wall_sequence.md`。

## 顶吸增量（2026-10-06）

新请求默认`suction_mode="auto"`：侧吸接触IK/抽离到位候选失败后，再检查顶升通道并尝试顶吸。可显式指定`side`/`top`；旧JSON省略该字段仍解释为side并保留原请求身份。`targets`保持侧吸参考Tool0目标合同；顶吸由该目标推导箱体目的位姿，再换算实际顶吸Tool0目标，记录在`carry_tool_targets`，不会将箱体旋转/平移到另一个目的地。

接触点、tool_to_box、40球载荷、稳定性本地上方向、缓存键、Viser均使用同一吸附模式。顶吸采用模型末端平面0.175×0.355m的内缩落点，升降轴垂直抬升35cm；从上方预抓取位沿同一路径下降到接触。搬运目标先预检完整180°转身，最后仍作原密集复核。

L03/R01顶吸完整周期实跑通过；整墙自动模式仍24/25，L02侧吸和顶吸均未得到合格接触IK。不能声称25/25。完整证据、CPU回放及GPU共享锁命令见`../../docs/motion233_curobo_top_suction.md`。
