# TCP任务输入与随机停车箱墙预览

本轮只验证输入反推/场景几何与可视化，不运行抓取规划。基于alfa_v3_curobo的feature/tcp-scene-contract-preview；旧完整任务链和后续实验保留，不在本阶段合并未经视觉验收的任务重构。

## 输入合同

每臂可为空；有任务时为footprint参考系下接触TCP完整SE(3)与side/top类型。工具约定沿用tool0，类型对应的tool_to_box转换不同（侧吸沿工具Z偏移箱深/2，顶吸沿工具Z偏移箱高/2，旋转也不同）。完整工具姿态必须包含绕法向的转角，才能唯一恢复完整箱体；预览由已知箱Pose生成TCP，再仅凭TCP与类型反推核对。

未来放置输入为导航到位后footprint下左右箱体最终SE(3)，与抓取TCP语义不同；当前预览只验证抓取/箱墙，不接入放箱或action。Motion自管权威关节反馈/阶段，从Home到Home；外部负责导航/吸放事件，仿真可瞬时确认，不在算法里直接控制电磁阀。

## 随机化和几何

保留原94双臂及25单臂演示任务选择。箱号/排列仅在模拟输入生成端使用，reconstruct_scene签名不接收箱号/任务排号。每次选择或点重新随机，箱体独立Roll/Pitch±2°、Yaw±5°；停车XY在半径.05m圆内面积均匀，Yaw±5°。不随机移动集装箱；场景一直固定world坐标，通过world_from_footprint转换TCP输入。

由TCP及抓取类型反推目标旋转箱体，用完整8角世界界限决定工作层：最低箱心附近2cm内视作同层，下方填到最低目标角高度，工作层顶取低层旋转箱体最高角；箱墙X厚度用旋转箱体X包络联合范围，左右覆盖当前集装箱内宽±1.2m。按每个目标旋转盒最左/右角各加2mm扣掉横向区间，剩余用实心cuboid填充，不按剩余箱子逐箱建模。目标箱单独可视化为完整旋转几何。

2cm同层容差是预览参数，不应在后续算法中偷偷视作已验收传感器高度合同。使用AABB的左右投影区间留孔而不是精确OBB布尔差；与用户“左右最远边界”定义一致。上下/前后旋转包络使用统一安全边界；当前不额外增加底部支撑缝隙，存在倾角接触边界需用户视觉检查。

绿色线固定world -X抽离方向；footprint下向量随停车Yaw变，不能误用footprint -X固定抽离。页面显示实际停车/每箱扰动及TCP参考系数值证据，勾选填充/剖视仅显示变化，不是修改规划碰撞模型。静态预览没有实际planning collision checker接入，不宣称抓取可行。

## 使用和证据

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3
/mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/v3_tcp_scene_preview.py --port 8152
```

模型与外观仍使用当前冻结模型/前端包；已知集装箱/初始状态在config/tcp_scene_preview.json，不依赖临时历史轨迹。最新生成样本保存在/tmp/tcp_scene_preview_20261018/latest.json，不把完整模拟箱号传入重建器。

test_tcp_scene_contract.py检查500个随机混合侧/顶吸输入的Pose反推、填充与目标分离、停车边界及世界抽离方向不变，另核对左右2mm留缝。视觉验收由用户在8152进行；通过前不继续把该场景生成器替换完整规划链路。

任务切换回归：启动命令增加--smoke-test，会用真实Viser/ReadOnlyScene渲染接口依次生成119任务各两次再退出。箱体SizedVisual保持常驻，切换只修改位姿与visible，不对其调用不存在的remove；普通辅助节点仍按其标准remove接口清理。
