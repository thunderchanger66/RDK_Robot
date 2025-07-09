#!/bin/bash

# 设置保存路径
MAP_DIR="/home/thunder/ros2_ws/src/startlocal/map"
MAP_NAME="my_map"

# 保存 pbstream 文件
echo "📦 保存 Cartographer 地图为 pbstream..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_DIR}/${MAP_NAME}.pbstream'}"

# 等待服务响应（可选）
sleep 2

# 保存 pgm 和 yaml 文件
echo "🗺️ 保存 Nav2 地图为 pgm 和 yaml..."
ros2 run nav2_map_server map_saver_cli -t map -f "${MAP_DIR}/${MAP_NAME}"
