#!/bin/bash
# CAN 接口初始化 - 启动实机前执行，防止 ENOBUFS (No buffer space available)
# 用法: sudo ./setup_can.sh [can0]

INTERFACE="${1:-can0}"

echo "Setting up CAN interface: $INTERFACE"

# 增大发送队列，默认 10 易导致 No buffer space available
ip link set "$INTERFACE" txqueuelen 256

# 若接口未启动则启动
if ! ip link show "$INTERFACE" | grep -q "state UP"; then
  ip link set "$INTERFACE" up type can bitrate 1000000
fi

ip -details link show "$INTERFACE"
echo "Done. Start robot with: ros2 launch alfa_robot_bringup alfa_robot.launch.py use_mock_hardware:=false controllers_file:=alfa_robot_controllers_can.yaml"
