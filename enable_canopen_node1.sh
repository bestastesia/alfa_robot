#!/bin/bash
# 使能 CANopen Node 1 (updown 关节)
# 用法: ./enable_canopen_node1.sh
# 前提: can3 已经 up
#
# SDO 帧格式 (8 bytes):
#   [cmd] [index_lo] [index_hi] [subindex] [data0] [data1] [data2] [data3]
#   cmd: 0x2F=1byte, 0x2B=2byte, 0x23=4byte, 0x40=read

CAN_IF=can3
NODE=01

echo "=========================================="
echo " CANopen Node 1 使能脚本"
echo " CAN: $CAN_IF, SDO TX: 0x601, RX: 0x581"
echo "=========================================="
echo ""
echo "请在另一个终端运行: candump $CAN_IF"
echo "按 Enter 开始..."
read -r

echo ">>> Phase 1: NMT Reset → Pre-Operational"
cansend $CAN_IF 000#81${NODE}
sleep 0.2

echo ""
echo ">>> Phase 2: TxPDO1 配置"

# 禁用 TxPDO1: 0x1800:01 = 0x80000181 (bit31=1)
echo "  [1] 禁用 TxPDO1 (0x1800:01 = 0x80000181)"
cansend $CAN_IF 601#2300180181010080
sleep 0.02

# 传输类型: 0x1800:02 = 1 (SYNC)
echo "  [2] TxPDO1 传输类型=SYNC (0x1800:02 = 1)"
cansend $CAN_IF 601#2F00180201000000
sleep 0.02

# 抑制时间: 0x1800:03 = 10 (1ms)
echo "  [3] TxPDO1 抑制时间=1ms (0x1800:03 = 10)"
cansend $CAN_IF 601#2B0018030A000000
sleep 0.02

# 清空映射: 0x1A00:00 = 0
echo "  [4] 清空 TxPDO1 映射 (0x1A00:00 = 0)"
cansend $CAN_IF 601#2F001A0000000000
sleep 0.02

# 映射 statusword: 0x1A00:01 = 0x60410010
echo "  [5] 映射 statusword (0x1A00:01 = 0x60410010)"
cansend $CAN_IF 601#23001A0110004160
sleep 0.02

# 映射 actual_position: 0x1A00:02 = 0x60640020
echo "  [6] 映射 actual_position (0x1A00:02 = 0x60640020)"
cansend $CAN_IF 601#23001A0220006460
sleep 0.02

# 映射数量=2: 0x1A00:00 = 2
echo "  [7] 映射数量=2 (0x1A00:00 = 2)"
cansend $CAN_IF 601#2F001A0002000000
sleep 0.02

# 使能 TxPDO1: 0x1800:01 = 0x00000181
echo "  [8] 使能 TxPDO1 (0x1800:01 = 0x00000181)"
cansend $CAN_IF 601#2300180181010000
sleep 0.02

echo ""
echo ">>> Phase 3: RxPDO1 配置"

# 禁用 RxPDO1: 0x1400:01 = 0x80000201 (bit31=1)
echo "  [9] 禁用 RxPDO1 (0x1400:01 = 0x80000201)"
cansend $CAN_IF 601#2300140101020080
sleep 0.02

# 传输类型: 0x1400:02 = 1 (SYNC)
echo "  [10] RxPDO1 传输类型=SYNC (0x1400:02 = 1)"
cansend $CAN_IF 601#2F00140201000000
sleep 0.02

# 清空映射: 0x1600:00 = 0
echo "  [11] 清空 RxPDO1 映射 (0x1600:00 = 0)"
cansend $CAN_IF 601#2F00160000000000
sleep 0.02

# 映射 controlword: 0x1600:01 = 0x60400010
echo "  [12] 映射 controlword (0x1600:01 = 0x60400010)"
cansend $CAN_IF 601#2300160110004060
sleep 0.02

# 映射 target_position: 0x1600:02 = 0x607A0020
echo "  [13] 映射 target_position (0x1600:02 = 0x607A0020)"
cansend $CAN_IF 601#23001602200078A60
sleep 0.02

# 映射数量=2: 0x1600:00 = 2
echo "  [14] 映射数量=2 (0x1600:00 = 2)"
cansend $CAN_IF 601#2F00160002000000
sleep 0.02

# 使能 RxPDO1: 0x1400:01 = 0x00000201
echo "  [15] 使能 RxPDO1 (0x1400:01 = 0x00000201)"
cansend $CAN_IF 601#2300140101020000
sleep 0.02

echo ""
echo ">>> Phase 4: NMT Start → Operational"
cansend $CAN_IF 000#01${NODE}
sleep 0.05

echo ""
echo ">>> Phase 5: CiA 402 状态机"

# Shutdown: 0x6040:00 = 0x0006
echo "  [16] Shutdown (0x6040 = 0x0006)"
cansend $CAN_IF 601#2B40600006000000
sleep 0.02

# Switch On: 0x6040:00 = 0x0007
echo "  [17] Switch On (0x6040 = 0x0007)"
cansend $CAN_IF 601#2B40600007000000
sleep 0.02

# Enable Operation: 0x6040:00 = 0x000F
echo "  [18] Enable Operation (0x6040 = 0x000F)"
cansend $CAN_IF 601#2B4060000F000000
sleep 0.02

echo ""
echo ">>> Phase 6: Profile Position 模式 + 速度参数"

# PP mode: 0x6060:00 = 1
echo "  [19] PP 模式 (0x6060 = 1)"
cansend $CAN_IF 601#2F60600001000000
sleep 0.02

# Profile Velocity = 10000 pps (0x00002710, LE: 10 27 00 00)
echo "  [20] Profile Velocity = 10000 pps ≈ 0.76 mm/s"
cansend $CAN_IF 601#2381600010270000
sleep 0.02

# Profile Acceleration = 5000 pps² (0x00001388, LE: 88 13 00 00)
echo "  [21] Profile Acceleration = 5000 pps²"
cansend $CAN_IF 601#2383600088130000
sleep 0.02

# Profile Deceleration = 5000 pps²
echo "  [22] Profile Deceleration = 5000 pps²"
cansend $CAN_IF 601#2384600088130000
sleep 0.02

echo ""
echo "=========================================="
echo " ✓ Node 1 使能完成！"
echo "=========================================="
echo ""
echo "--- 验证命令 ---"
echo ""
echo "读取 statusword (应返回 0x0637 = Operation Enabled):"
echo "  cansend $CAN_IF 601#4041600000000000"
echo ""
echo "读取当前位置:"
echo "  cansend $CAN_IF 601#4064600000000000"
echo ""
echo "--- PDO 测试 ---"
echo ""
echo "1) 发送 SYNC (触发 TxPDO1 响应 0x181):"
echo "   cansend $CAN_IF 080#"
echo ""
echo "2) 发送位置命令 (RxPDO1: controlword=0x003F, target=0):"
echo "   cansend $CAN_IF 201#3F0000000000"
echo "   cansend $CAN_IF 080#"
echo ""
