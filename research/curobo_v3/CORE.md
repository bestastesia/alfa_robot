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

原FullCyclePlanner缓存按几何身份及负载内容失效；仅反馈时间或关节值变更不重建静态世界检查器。
原基类同任务更新障碍位置、基座位姿、附着物或碰撞策略、箱球配置时会重建。
当前优化入口通过PersistentCollisionPlanner按拓扑常驻资源，几何变化使用update_world更新而非重建；
负载/机器人拓扑改变仍创建新资源。容量40 cuboid、无mesh、单实例串行，详见OPTIMIZED_CYCLE.md。
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

原FullCyclePlanner完整周期支持底盘在map原点、头部两轴固定零位、两个不同的轴对齐标准箱；
原基类不支持已携箱启动、单臂全周期或任意基座位姿全周期。
当前SingleArmPlanner研究入口支持1/2活动臂，PlanRequest目标必须精确对应活动TCP，
只给活动臂加载负载球，119任务和整机运输终点包络硬验收见OPTIMIZED_CYCLE.md。状态/场景层可以表达这些状态，
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
