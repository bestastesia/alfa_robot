# 解析64接触与多根完整规划同步

2026-10-11，依据交接`handoff-oFc145.md`定向迁移至alfa_robot_v3的alfa_v3_curobo分支。
源alfa_robot_v3_curobo及其未提交研究保持不变；没有新增工作树。

## 当前默认研究方案

两臂各抓独立箱子，规划15轴（共享updown＋双七轴），不是闭链。头部锁零；底盘仅在
顶吸前进和转身动作中变化，不接入通用18轴运输搜索。
Home：updown=-0.3m，左[155,-90,20,90,-90,-40,0]°，右为相反数。
仅研究代理named poses更新，ROS生产Home不改。

94组选项：上三层60侧吸、下两层34顶吸，同层/邻层但不跨第三与第四层；动态填充
较低目标层及所有更低区域。箱墙底部抬1mm。

1. 顶吸检查底盘前进0.3m；侧吸不前进。
2. 遍历updown[-1,0]每5cm及左右ψ[-180,170]每10°；同一updown组合解析分支。
3. 限位、球碰撞、FK≤2mm/1°、1e-6去重，按原Home距离选最低64。
4. 对64候选分别固定updown/ψ解析后撤35cm，保留全部通过的抽离终点。
5. 每根从冻结放置解集按距离取近邻512，17点FK旋转评分，最多8个差异目标；目标合并去重。
6. GPU算子加速自写多根Connect联合搜索，累计搬运搜索预算2s。当前多根不启用Informed椭球。
7. 被选根检查Home→3cm预抓取、解析贴近、抽离、搬运及空载回Home；失败排除相关根/目标继续剩余预算。
8. 到放置、yaw180°、释放删箱、yaw转回、空载Home。全部成功才播放。

保留目标已有distance_metric：updown10cm等于普通关节15°；接触排序/搜索均统一，
不是照抄源接触排序的5 rad/m。碰撞采样/密化仍采用至少5 rad/m的升降分辨率。
撤回的腕部权重、joint4惩罚、新Home、预抓取关键帧优化不接默认。

| 模式 | 局部工具位置 | wxyz |
| --- | --- | --- |
| 侧吸 | [0.55,±0.425,1.4]m | [0,0,0,1] |
| 顶吸 | [0.55,±0.425,1.0]m | [0,1,0,0] |

目标是完整吸盘Pose，真实tool_to_box不改。顶吸内部局部世界对应底盘前进0.3m，输出map
的放置x=0.85m；内部快照/预测事件局部语义尚未生产化，不能直接用于实机接口。

## 碰撞与质量合同

动态结构/臂/箱仍用球模型，箱40球为内接近似。底盘对外世界保留71球；底盘内部
对臂/负载用base_link系4冻结长方体，查询排除base_link与arm_carriage连接区域。
实现字段base_mesh_layer是查询接口名，当前内部层不做STL复查。
放置解集整机/负载包络≤1m，接触解不加该约束。
预抓取检测目标箱；最后3cm排除两目标；抽离途中延后负载碰撞至35cm终点。
携箱转身和空载转回按用户研究指令跳过碰撞检查，动作保留；不宣称全流程每帧无碰撞。
运输旋转软代价与额外旋转≤90°、回退proxy≤15°、弧偏离proxy≤45°细验保留。
这些是几何研究条件，不是吸力/动力学、速度加速度或实机安全验收。

## 工程收口

入口根据__file__定位当前checkout，不导入时chdir；使用prepare_roots方法而非全局函数
替换，避免污染其他planner实例。接触/过滤atlas缓存身份包含场景几何、模型、目标Pose、
atlas哈希、cuboid定义、payload和接触网格。修改放置policy后逐行FK验证atlas一致性，
不自动用旧解集，也不自动重生成。快照仍限定研究固定任务，不宣称并发生产服务。
失败默认空frames/phases/payload/joint_names，切换/失败恢复不重用旧cached_selection。

记录解析枚举/碰撞FK/总生成以及检查器/全部抽离/匹配/多根搜索/前后验证/组装计时。
cached solve仅返回已生成接触候选，约0ms不等于解析生成成本；setup与生成分项重叠，
不能直接求和。首次含GPU/atlas初始化，冷热态需分开比较。

Home→预抓取末端可能多转一圈，目前没有该段末端旋转代价，未声明解决。

## 冻结输入和源研究证据

config/research_asset_manifest.json保存主要运行资产哈希；两份npz是必要输入，随PR提交。
config/placement_task_policy.json与实际atlas逐行FK对齐，config/base_link_collision_frozen.json
保存71底盘球，generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json
保存内部4长方体。未提交.so或机器路径，prepare_checkout继续负责路径重定位与解析桥编译。

源报告汇编在research_evidence/，保留各自策略、日期和原结论。源侧吸60/60及较早91/94
不是迁移后94任务重验。迁移结果摘要在generated/research_sync/validation.json；原始大轨迹
保留/tmp/curobo_sync_validation及源目录，避免把数十MB研究轨迹塞入PR。

## 启动与验收

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/prepare_checkout.py \
  --compiler g++
PYTHONDONTWRITEBYTECODE=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
  /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_mixed_placement_demo.py \
  --port 8140
```

8140为迁移入口，无预载结果；点击在线计算。源8137和所有现有研究页不改。当前本机
使用用户级临时服务alfa-curobo-sync-8140.service保持后台，不随工具调用退出；服务停止
命令systemctl --user stop alfa-curobo-sync-8140.service，仅针对本次新服务。

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3
PYTHONDONTWRITEBYTECODE=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
  /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python tools/v3_research_regression.py \
  --output-dir /tmp/curobo_sync_validation \
  --repeat 1
```

该命令检查两种模式及代表中列任务的完整周期、移动模型FK、真实附着变换、Home误差、
base转换和阶段/帧连续性。不要同时运行多个计算任务导致GPU竞争改变计时。
