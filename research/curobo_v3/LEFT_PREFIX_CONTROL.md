# 左臂初始至接触录制回放

feature/ipc-left-prefix-control，用户已确认只回放左7轴，不控制升降/右臂、不承诺TCP到原箱面。基于contact-roll-alignment实验，未PR合并或实机运动验收。

## 已完成

- generated/left_prefix_control_20261018/prefixes.json保存六组左臂前缀。截断在attach之前最后contact_approach帧，完全排除吸附、抽离、运输、放箱、释放；回Home只逆序同一前缀。
- 工控机/home/ar/alfa_left_prefix_control_20261018存放独立标准库脚本和数据；alfa-left-prefix-control.service运行在127.0.0.1:18122，ROS_DOMAIN_ID=77。没有更改RT控制源码、没有自动使能。
- 本机alfa-left-prefix-tunnel-8155.service将localhost:8155转发到上述后端。前端可选择六组任务、查看真实反馈/目标角度、最高速度、预计时长、获取/释放18121控制权及取消。内嵌8156只读Viser（alfa-left-prefix-live-8156.service），用实时反馈驱动左臂；绿球标记所选接触目标，右臂隐藏、升降-.3m仅显示参考，反馈过期立即提示，不伪造实机状态。
- 最新控制台反馈左[65,-75,20,-90,0,40,90]°，J3/J6与自制模型同号。旧反号映射已经取消。
- 后端通过18121既有认证建立独立会话、SSE心跳及lease互斥，令牌只在工控机读取、不会发给前端或记录内容。实际软限位从18121读取，每次发送再检查；不跳过起点/新鲜反馈/停稳/active/ENABLED/Action服务门禁。
- 停止调用当前Action目标的取消接口，等待控制器取消结果，停稳后才允许沿原前缀逆序回Home；不是硬件急停。页面失去心跳或控制权丢失时请求取消，未结束前不释放lease。

## 2026-10-19接口已对齐

已读取实际ROS图：/left_arm_jtc/joint_trajectory没有订阅者，left_arm_jtc仅订阅/parameter_events。18121真正执行接口为/left_arm_jtc/follow_joint_trajectory Action。

用户确认沿用既定FollowJointTrajectory Action，已将后端切换到/left_arm_jtc/follow_joint_trajectory，并接入接受/拒绝、执行结果、错误码和取消。已部署工控机服务并更新8155前端，未修改RT控制器。旧--topic参数移除，使用--action（默认即上述接口）。

已确认真实Action服务可见、反馈/控制器可读；本轮未使能、未接管其他终端、未发送实机动作，实机motion_publications=0。若18121持有控制权，应先在那里释放，再到8155获取。若机器人不在记录Home，不会自动规划回Home或硬连线，须先用原控制台调整到左[65,-75,20,-90,0,40,90]°，再释放控制权。

## 验证

六项标准库单元测试通过（导出截断、仅7轴/映射、返回方向、速度/加速度/jerk、拒绝后段、状态门禁）。
实际ROS2收发在隔离Domain231完成，脚本integration_smoke.py拒绝在其他Domain运行。最新ActionServer测试包含成功、逆序回Home、取消、拒绝及执行失败；已接受5个左7轴目标，最终Home误差0，退出码0。
证据/tmp/left_prefix_action_smoke_20261019.log；此前topic测试仅为历史模拟，不是实机执行验收。

## 2026-10-19短消息与时间上限

用户确认单程硬上限60秒、峰值关节速度15°/s。旧发送端20ms加密和每加密点停稳已移除；只合并7D近乎共线、进度单调的插值点，误差≤2e-7rad（约0.000012°），保留折点，不作绕障shortcut。每个折点零速/零加速度，RT自行五次插值，同一条关节直线、无过冲；升降/右臂冻结的实机碰撞仍非本工具保证。

五次进度10s³-15s⁴+6s⁵，单段时长按1.875·最大关节位移/速度、sqrt(5.773503·位移/加速度)、cbrt(60·位移/jerk)三者最大值计算；上限15°/s、30°/s²、150°/s³。后端发送前及构造Action前再次验峰值速度、加速度、jerk、60秒和200点硬门禁。运行超过核验时长0.5秒或总60秒时请求取消，实际停止仍需控制器确认。

|任务|发送点|15°/s单程预计|
|---|---:|---:|
|L18/R22|10|16.4秒|
|L4/R2|17|25.5秒|
|L4/R5|17|31.6秒|
|L7/R5|20|38.7秒|
|L8/R7|21|38.2秒|
|L9/R1|17|27.1秒|

实际发送按新鲜反馈补第一个衔接点后重新核验，因此界面预计与最终发送可能略有差别；超限就拒绝，不提高速度强行赶时间。降速至1°/s可能因超过60秒被拒。摘要compact_timing.json；9项单测覆盖合并不切拐点/不跨反转、六组原点到新路径误差、五次插值内部采样峰值及超时/超点拒绝。隔离Action重测通过，/tmp/left_prefix_compact_smoke_20261019.log。没有重新使能或重发真实动作。

此前真实掉使能仍需RT侧处理：本次减少输入点数不能当作实机故障恢复或实机新轨迹验收。运行期信息已保留于LEFT_PREFIX_FAULT_20261019.md；黑匣子另确认发送后周期8.381ms及随后的总线故障，RT实时接收全轨迹重排是已发现风险。

## 重启

工控机服务已设置完整ROS环境，令牌文件会自动定位当前18121进程：

```bash
ssh ar@192.168.100.40 'systemctl --user restart alfa-left-prefix-control.service'
systemctl --user restart alfa-left-prefix-tunnel-8155.service
systemctl --user restart alfa-left-prefix-live-8156.service
```

Action、反馈、控制器、使能、停稳和控制权均满足后才可发送。原18121使能和现场停止流程保持独立。

## 原控制台重启后的会话恢复

2026-10-19修复：18121进程重启会使旧Cookie失效，旧脚本获取控制权持续409。当前HTTP401后自动发现正在运行的控制台令牌文件并重新认证，同时重读实机软限位；只恢复会话，不自动获取控制权或使能。控制器list_controllers未响应2秒后撤销待定查询并重试，页面区分“查询过期”和“RT报告未激活”，状态包含controller_age_s/console_age_s及错误信息。

11项单元测试通过，含401恢复后单次重试、inactive与stale区分；8155实际获取/释放控制权成功，实机发送计数0。排查当时18121同样显示未使能/left_arm_jtc未激活，RT反馈IDLE、查询年龄约0.3秒，因此这部分并非显示误报。应先在18121使能成功，再释放控制权并切回8155；本脚本不会自动替用户使能。
