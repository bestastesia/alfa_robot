# PLC 轨迹队列 Modbus 测试工具

这个目录用于在不启动 ROS / MoveIt 的情况下，直接测试 SEV-7 中的
`Modbus双臂轨迹接口 V0.2-工程对齐版`。

目标是先验证：

- Modbus TCP 能连接 PLC。
- `prepare/write/commit/start` 握手可用。
- PLC 能接收 5 点、20 点、240 点轨迹。
- PLC 能通过 PTHC UserDefined 执行双臂 12 轴点表。
- 小幅多轴同步轨迹可以同时开始、同时结束。

## 协议参数

默认值来自 V0.2 文档：

```text
PLC IP: 192.168.1.88
Port: 502
Unit ID: 1
状态区: offset 100~123
包头区: offset 200~239
点数据区: offset 240~359
每包: 5 点，120 registers
每点: 12 轴，每轴 DINT mdeg，高字在前
```

## 安全前提

测试运动前必须知道 12 轴当前安全角度，并传给 `--base`。

即使只测一个轴，其他 11 个轴也必须填当前角度，不能默认填 0。

脚本在 `--upload` 时会强制要求提供 `--base`，除非使用 `--csv` 提供完整 12 轴点表。

`--base` 支持两种写法：

```bash
--base "10,53,51,111,-88,111,-88,111,0,0,0,0"
--base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0"
```

## 1. 只读状态

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --status
```

期望能看到：

```text
ready=1 ... fault=0 err=0(NO_ERROR) activeBank=...
```

## 2. 生成轨迹但不写 PLC

保持当前位置 5 点：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario hold \
  --points 5 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0"
```

导出 CSV 给人工检查：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario mixed-delta \
  --points 20 \
  --small-delta 0.1 \
  --large-delta 0.6 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --export-csv /tmp/plc_mixed_delta_20.csv
```

## 3. prepare/write/commit 保持轨迹，不 start

先按同事已跑通脚本的方式，只做 `prepare + write 1 包 5 点`，不 commit、不 start：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario hold \
  --points 5 \
  --cycle-ms 100 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --write-only \
  --verbose
```

如果 5 点已经 ACK，再单独 commit 当前 Bank：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --commit-existing \
  --target-bank 1 \
  --verbose
```

`--target-bank` 要和刚才写入的 Bank 一致；不填时脚本会尽量按 `writeBank` 推断。

## 4. start 保持轨迹

PLC 会执行刚才 commit 的 5 个完全相同的点，理论上不明显运动：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --start-existing \
  --target-bank 1 \
  --verbose
```

如果想一条命令完成 `prepare/write/commit/start`，再使用：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario hold \
  --points 5 \
  --cycle-ms 100 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --verbose
```

## 5. 单轴小幅运动

只让 Axis1 从当前位置线性增加 `0.3 deg`，其他轴保持：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario single-axis \
  --axis 1 \
  --delta 0.3 \
  --points 20 \
  --cycle-ms 100 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --verbose
```

## 6. 多轴同步：正弦小幅测试

12 轴都有小幅变化，适合观察是否同步开始/结束：

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario sync-wave \
  --amplitude 0.2 \
  --cycles 1 \
  --points 40 \
  --cycle-ms 100 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --verbose
```

## 7. 多轴同步：非均匀段长/非均匀轴差测试

这个测试专门覆盖两个问题：

1. 轨迹里相邻点之间，有的段变化小，有的段变化大。
2. 同一段里，不同关节的变化量不一样，有的轴变化大，有的轴变化小。

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario mixed-delta \
  --small-delta 0.1 \
  --large-delta 0.6 \
  --points 40 \
  --cycle-ms 100 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --verbose
```

先用 `large-delta 0.3~0.6`，确认安全后再逐步放大。

## 8. 240 点长轨迹测试

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py \
  --scenario mixed-delta \
  --small-delta 0.1 \
  --large-delta 0.6 \
  --points 240 \
  --cycle-ms 50 \
  --base "1:10,2:53,3:51,4:111,5:-88,6:111,7:-88,8:111,9:0,10:0,11:0,12:0" \
  --upload \
  --verbose
```

## 急停/停止

V0.2 文档里 `stopToggle` 当前主要为预留，因此这个命令只能作为接口测试，
不能当成可靠急停。

```bash
python3 scripts/plc_trajectory_queue_test/plc_queue_client.py --stop --yes-write
```

真正急停仍然应使用现场硬件急停/驱动安全链路。

## 重要限制

- 当前 PLC 文档说明还没有通过 Modbus 映射 `actualQ1_mdeg..actualQ12_mdeg`。
- 因此本工具第一版只能根据 `running/done/fault/errorCode/executedLine` 判断执行结果。
- 不能用它确认实际角度误差；实际角度闭环要等 PLC 映射 `actualQ1~12` 后再做。
- `packetCrc32` 当前按文档填 0，暂不校验。
