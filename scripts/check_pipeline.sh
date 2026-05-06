#!/bin/bash

# 获取脚本所在目录的绝对路径作为工作区路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_DIR="${SCRIPT_DIR}"

echo "=========================================="
echo "          感知系统状态一键自检            "
echo "=========================================="

echo "正在加载环境变量..."
source /opt/ros/*/setup.bash
source ${WS_DIR}/install/setup.bash

# 定义检查话题函数
check_topic() {
    local topic_name=$1
    local desc=$2
    
    echo -n "检查 ${desc} (${topic_name}): "
    
    # timeout 3秒，如果有任何数据(一帧)发出则返回成功
    timeout 3 ros2 topic echo ${topic_name} --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "\033[32m[正常 有数据]\033[0m"
    else
        echo -e "\033[31m[异常/无数据]\033[0m"
    fi
}

echo "--- 1. 传感器层 ---"
check_topic "/camera/camera/color/image_raw" "D455 相机图像"
check_topic "/camera/camera_info" "D455 相机内参"
check_topic "/livox/lidar" "MID360 雷达点云"
check_topic "/livox/imu" "MID360 雷达IMU"

echo "--- 2. LIO 里程计层 ---"
check_topic "/cloud_registered_body" "Fast-LIO 点云输出"

echo "--- 3. 感知位姿层 ---"
check_topic "/box_perception/result" "箱体感知结果包"
check_topic "/target_odometry1" "距离最近箱体 6D Pose"

echo "=========================================="
echo "自检完成！"
echo "💡 提示："
echo "   1. 如果 [异常/无数据]，请去查看 /tmp/ 目录下的对应报错日志。"
echo "   2. 检查 /target_odometry1 时，请确保相机视野内有可以被识别的箱子。"
echo "=========================================="
