# Fast全流程评审版本

2026-11-10更新：当前PR已在PR80基础上加入双臂LM64控制点32轮后处理。
部署、模型/碰撞、依赖校验及实时计算/可视化命令请使用[FAST_DEPLOYMENT.md](FAST_DEPLOYMENT.md)。
下方11月9日记录及绝对工作区命令是PR80历史，不是本次新增后处理的完整部署说明。

本PR把已讨论的方案整理为可运行入口，供人工审核；不合并PR，不发送硬件。
已知问题保留，不把失败候选写成成功，也不在本次强制修复。

## 三层流程

1. 轨迹规划：固定Home → 多个解析吸附IK → 预测对应抽离终点 → 多根、多运输目标RRT选组合 →
   仅选中候选完整15轴LM抽离 → 从实际LM终点连接同一运输目标 → 验证Home接近/贴近 →
   15轴LM放置（末态关节自由求解）→ 释放 → RRT回Home。失败尝试及选中LM诊断仍按原链路记录。
2. 轨迹优化：仅`approach/transport/return_home`运行Fast几何TrajOpt；32控制点、50轮LBFGS、
   单种子、几何形状代价系数100、CUDA Graph。抽离/放置/贴近不做TrajOpt。
3. 时序优化：全部六运动段运行原生Fast `TrajectoryTimeOpt`，持箱段加每箱10kg质量/质心/惯量。
   单臂七轴连续力矩为`[294,294,178,178,87,52,10] Nm`，速度60°/s，jerk 500rad/s³，
   限制系数0.995。Updown仍用有限大界近似不限；加速度界也取有限大值，不代表已确认硬件参数。

RRT保留自研多根/多目标策略，FK和球体碰撞由Fast运行时执行，并未改成官方单起终点规划器。
LM采用Fast运行时的`SeedIKSolver`，两段均无RRT回退。资源/静态运输IK库可以常驻，
每次实时请求从头做任务规划及优化，不读取先前已完成任务作为答案。

## 碰撞粒度与膨胀

| 部位/关系 | 本PR采用的几何 |
| --- | --- |
| `arm_carriage`的祖先：`base_footprint → base_link` | footprint无实体；base_link底盘和固定立柱使用原四盒 |
| 固定结构与手臂/头部/携箱 | 原BaseCuboidLayer球-盒检测 |
| 固定结构与外部障碍 | 新增盒-盒15分离轴检测；移动底盘时同步平移及Yaw旋转 |
| 升降移动架 | 原20球，随Updown运动，不换盒 |
| 双臂、独立头部分支、携箱 | 原球模型，携箱每箱40球 |

原base_link71个球槽位保留但半径置负，不再参与Fast入口的自碰撞/世界球查询；不是盒和71球叠加。
场景改变更新盒体世界数据；固定基座下常量盒-世界关系只算一次，移动基座按查询姿态计算。

自由RRT的外部障碍查询球半径、底盘盒半尺寸都乘1.10，属于百分比膨胀。
IK接触、LM、自碰撞、内部四盒关系、地面及TrajOpt保留基准几何。
世界碰撞仍沿用原1mm容差，不能把它解释成10%膨胀。
搜索节点/边、LM节点检查保留；没有新增返回路径密化、0.25°、后处理/最终整段碰撞门。
四盒定义沿用`generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json`；
其历史`applied_to_planner`元数据不是运行时开关，实际启用位置见`v3_fixed_body_boxes.py`。

## 已知问题（本次保留）

- **TrajOpt可行性标记仍有问题**：原`project_base_and_ground`同时进入代价与约束；
  base 5mm/ground 2mm间隙不足也可能标为失败，不等价于实际穿透。
  本次撤回了软硬语义分离修正，保留原判定，后续统一处理；不据失败标记直接推断碰撞位置。
- **迭代仍可能绕路/结果变差**：形状代价100倍和规划膨胀不构成质量或成功保证。
  三个本轮任务都存在至少一个TrajOpt失败阶段，候选与TimeOpt数据照常保存供查看。
- **冷启动及新资源耗时明显**：首次求解/Graph捕获/新负载拓扑可能数秒至数十秒；
  没有“完整流程稳定3秒”保证。墙钟、规划、纯求解、运动时长分别列出。
- **原生B样条拟合是近似表示**：RRT32控制点初值、LM/贴近128控制点拟合可能偏离输入；
  报告关节及Updown拟合偏差，不把它写成严格逐点相同，也没有补加被取消的后验碰撞门。
- **Fast新入口仅双臂**：旧单臂入口未迁移，不以双臂测试冒充全117任务成功率。
- **约束诊断可能缺失**：Fast结果`metrics=None`时保留success及`native_constraints_unavailable`，
  不编造逐项分数。
- **兼容日志命名仍有歧义**：部分继承层仍输出“解析抽离/固定Updown”等旧文字；
  实际完整抽离由`FrozenCyclePlanner.analytic_extract`兼容方法调用15轴LM，以上旧文字不代表算法回退。
- **历史模型/回放测试尚有4项不一致**：旧抽离profile、旧回放独立URDF位置误差、
  J6与旧模型的轴关系、旧Home断言；本次没有刷新旧回放或修改断言让其通过。
  属于待统一核对项，详见验证记录。未重跑全任务，未做实机验收。

## 本轮验证

RTX4060 Laptop 8GB，CPU线程2，同一进程按85→86→38计算，每箱10kg。
完整数据在本地`generated/fast_three_layers_20261107/`，提交的紧凑记录为
`generated/fast_release_20261109/validation.json`。全部18段TimeOpt成功且4001点动力学采样通过；
三个任务均保留完整候选，整体`success=false`反映TrajOpt失败，不是合格轨迹声明。

| 任务 | 规划计算 | 本次计算墙钟 | 候选运动时长 | TrajOpt失败阶段 |
| --- | ---: | ---: | ---: | --- |
| L4/R1顶吸 | 14.180s | 54.730s | 16.036s | transport |
| L4/R2顶吸 | 4.187s | 5.953s | 15.069s | approach、transport、return_home |
| L18/R11侧吸 | 3.847s | 14.454s | 14.521s | approach |

初始化另2.094s；第一个任务含冷启动，第三个任务改变负载布局会创建新资源，不能称稳定均值。
CPU/结构回归：32通过、1跳过、4项上述历史一致性失败。`git diff --check`通过。

## 启动本PR的本地评审检出

依赖已准备的cuRobo Fast隔离环境、现有研究模型及`alfa_motion_viser`。官方源码基线配置为
`76c2f4b6d325b4e76696939de1faa6c14d409e0a`；当前本地源码为无`.git`展开目录，
因此同时记录实际关键源码SHA256，不能把配置SHA当成已执行Git身份校验。
`prepare_checkout.py`为原有资产定位/解析库编译入口；它会更新本检出的资产绝对路径。

```bash
cd /home/li/.codex/worktrees/curobo-fast-box-pipeline/alfa_robot && \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
research/curobo_v3/tools/prepare_checkout.py --compiler g++
```

```bash
cd /home/li/.codex/worktrees/curobo-fast-box-pipeline/alfa_robot && \
PYTHONPATH=/mnt/mydisk/ALFA/curobo_timeopt_v090:/home/li/.codex/worktrees/curobo-fast-box-pipeline/alfa_robot/research/curobo_v3/tools \
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python \
research/curobo_v3/tools/v3_fast_live_viewer.py --port 8290
```

打开`http://127.0.0.1:8290/`后选任务，点击计算才开始规划；支持全流程TimeOpt时间轴播放、
阶段跳转及左右七轴v/a/jerk/力矩曲线，显示失败标记并保留本次候选。
旧8275、8282、8289服务没有在本PR准备过程中被替换或停止。
