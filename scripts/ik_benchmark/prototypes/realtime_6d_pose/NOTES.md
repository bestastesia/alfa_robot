# 原型问题与当前答案

日期：2026-08-22。

## 问题

不启动或模拟 rt-control 时，能否把交互式 6D Pose 经 Motion 安全门后形成的10/30 Hz正式
滚动消息完整显示出来，让联调双方先确认 Motion 输出的数据形状和节拍？

## 当前实现答案

- Motion 从项目当前基线初始化；每帧仍执行解析 IK、关节突变和关节边连续碰撞检测。
- `rolling_suffix.py` 是无 ROS 的可移植协议逻辑：Hermite 采样、accepted splice、非伺服轴
  future 延续、500 ms/100 ms suffix 和保守包络整形。
- `motion_rolling_preview.py` 以10/30 Hz在安全 topic 发布正式 `RollingJointTargetBatch` 类型，
  同时展开每批6个未来点供 Rerun 查看。
- launch 不创建任何 `/rt/*` endpoint，也不发布 `/joint_states` 或模拟机器人跟踪。
- 当前 suffix 只从上一条 TX 拼接，验证的是 Motion 输出连续性，不能解释成 RT 接受状态。
- 响应修复后，固定目标沿同一个绝对时间profile持续前进，目标变化时才从精确splice q/v重规划；
  不再把每个500ms滚动窗口都当作零速终点。10°阶跃在Motion预览包络下约0.90s到达95%。
- Rerun 紫色机器人是滚动消息末点预览，不是实际机械臂反馈。

## 仍需现场回答

- 真实联调仍需由 session open 返回 controller/session/client 身份和替换前沿；预览 UUID 不能使用。
- 真正接入时，suffix 拼接基线必须由 rt-control ACK/state 推进，而不能使用“上一条已发送”。
- RViz/Rerun 壳在消息形状和节拍确认后应删除或迁入正式诊断工具。
