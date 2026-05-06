#!/bin/bash
# 零差云控电机滑块测试（仅 can0 Node 1,2）

echo "=== 零差云控电机滑块测试 ==="

# 1. 初始化 CANable2
DEV="/dev/ttyACM9"
echo "初始化 CANable2: $DEV"
echo -e "C\r" | sudo tee $DEV > /dev/null
sleep 0.1
echo -e "S8\r" | sudo tee $DEV > /dev/null
sleep 0.1
echo -e "O\r" | sudo tee $DEV > /dev/null
sleep 0.5

# 2. 创建 CAN 接口
sudo slcand $DEV can0
sleep 2
sudo ip link set can0 up

# 3. 检查接口
ip link show can0

# 4. 启动 ROS2 节点
echo ""
echo "启动 ROS2..."
source /mnt/mydisk/ALFA/alfa_robot/install/setup.bash

# 启动 controller_manager
ros2 launch alfa_robot_bringup alfa_robot.launch.py \
  use_mock_hardware:=false \
  robot_controller:=zeroerr_slider_controller \
  runtime_config_package:=alfa_robot_bringup \
  controllers_file:=zeroerr_slider_controllers.yaml \
  use_joint_gui_control:=true &

echo ""
echo "启动桥接节点..."
ros2 run alfa_robot_bringup zeroerr_slider_bridge.py
