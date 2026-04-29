#!/bin/bash
# pick_ik 一键部署脚本
# 在新机器上运行此脚本即可完成 pick_ik 的安装和编译

set -e

echo "=========================================="
echo "pick_ik 部署脚本"
echo "=========================================="

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS 环境未设置，请先 source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS 版本: $ROS_DISTRO"

# 安装系统依赖
echo ""
echo "[1/3] 安装系统依赖..."
sudo apt update
sudo apt install -y librange-v3-dev catch2

# 获取工作空间路径
WORKSPACE=$(dirname "$(readlink -f "$0")")
cd "$WORKSPACE"

# 检查 pick_ik 是否已存在
if [ ! -d "src/pick_ik" ]; then
    echo ""
    echo "[2/3] 克隆 pick_ik 源码..."
    mkdir -p src
    cd src
    git clone -b main https://github.com/PickNikRobotics/pick_ik.git
    cd "$WORKSPACE"
else
    echo ""
    echo "[2/3] pick_ik 源码已存在，跳过克隆"
fi

# 编译
echo ""
echo "[3/3] 编译 pick_ik..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select pick_ik \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

echo ""
echo "=========================================="
echo "pick_ik 部署完成！"
echo "=========================================="
echo ""
echo "使用方法:"
echo "  source $WORKSPACE/install/setup.bash"
echo "  ros2 launch alfa_robot_moveit_config demo.launch.py"