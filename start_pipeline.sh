#!/bin/bash

# 获取脚本所在目录的绝对路径作为工作区路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="${SCRIPT_DIR}"

echo "=========================================="
echo "      一键启动感知系统 (无可视化模式)     "
echo "=========================================="

echo "[0/4] 正在加载环境变量..."
source /opt/ros/*/setup.bash
source ${WS_DIR}/install/setup.bash

# 捕获 Ctrl+C 信号，以便退出时能自动清理所有后台启动的节点
trap "echo -e '\n[退出] 正在关闭所有节点...'; kill 0" SIGINT

echo "[1/4] 启动 MID360 雷达..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
sleep 3

echo "[2/4] 启动 D455 相机..."
ros2 launch realsense2_camera rs_launch.py > /tmp/camera.log 2>&1 &
sleep 3

echo "[3/4] 启动 Fast-LIO (rviz:=false)..."
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false > /tmp/fast_lio.log 2>&1 &
sleep 3

echo "[4/4] 启动 箱体 6D Pose 感知..."
ros2 launch box_perception perception.launch.py > /tmp/perception.log 2>&1 &

echo "=========================================="
echo "    所有节点已在后台启动！"
echo "    - 相机日志: /tmp/camera.log"
echo "    - 雷达日志: /tmp/lidar.log"
echo "    - LIO 日志: /tmp/fast_lio.log"
echo "    - 感知日志: /tmp/perception.log"
echo "    保持此终端开启，按 Ctrl+C 可一键结束所有程序。"
echo "=========================================="

# 等待后台进程，阻塞当前脚本
wait
