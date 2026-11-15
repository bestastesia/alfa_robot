# 2026-10-19左臂前缀发送后掉使能：只读诊断

未重新使能、未重发动作、未修改RT配置。8155真实状态：motion_publications=1，L18/R22 forward，11124点/218.871秒，Action aborted（status6、error_code=-1），错误为Current goal cancelled during deactivate transition。

## 证据时间线（Asia/Shanghai）

- 12:01:00.803：内核EtherCAT报告0-7 AL 0x001A Synchronization error，降SAFEOP；随后0-1至0-6也报相同同步错误。这发生在本次Action前约6秒。
- 12:01:06.536：left_arm_jtc收到新Action目标，12:01:06.537接受。
- 12:01:06.559：guard状态记录左J1～J6 status_word=0x1208，J7短时为0，enabled=False。本次没有guard TRIP tracking error日志；guard历史最大跟踪误差0.299°，不能当成本次瞬时误差。
- 当前只读DynamicJointState：7轴status_word=0x1208、error_code=0xA000；enable_manager为FAILED/fault_detected，首个故障left_joint1。RT自带zeroerr_cia402_csp.yaml注释将0xA000标注为EtherCAT communication abnormal，不能当作机械超限码或证明所有故障均因某个轨迹点产生。
- Action在控制器deactivate时中止。之后RT明确requires restart/power-cycle before enable；没有替用户重启或复位。

## 结论边界

直接链路是驱动故障→enable_manager失败/控制器停用→Action中止。不是起点校验拒绝，也没有证据证明机械臂走到某个碰撞点才停。按当前定时，约22ms时仍在初始0.1秒静止点内。

EtherCAT同步错误在发送前已经发生，因此不能简单归因为此次轨迹。但本次一次发送11124点，是新接入相对原18121点到点的明显差异，消息解析/轨迹接收是否放大实时抖动仍需RT工程师用周期时延和驱动错误记录核对，尚未排除。不能仅靠现有日志把责任认定为硬件或认定我们的发送绝无影响。

下一步先由RT侧确认0x001A/0xA000同步与通信故障的根因和恢复条件，再讨论是否减少轨迹点或改发送策略；不关闭故障保护、不自动重试。

## 证据位置

工控机/home/ar/rt-control-electri127/runs/motion-20261019-030424/的launch.log、guard.log、dmesg.log；本机副本/tmp/left_prefix_fault_20261019/。后端journalctl --user -u alfa-left-prefix-control.service记录唯一发送及Action中止。当前8155仍因RT状态FAILED禁止发送。
