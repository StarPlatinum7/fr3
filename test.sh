#!/bin/bash

# 启动 roscore
roscore &
sleep 5 # 等待 roscore 完全启动

# 更改串口权限
sudo chmod 666 /dev/ttyUSB0

# 运行 ROS 节点
rosrun force_msg force_msg_node &
sleep 2 # 确保节点启动

# 运行 Python 脚本

/usr/local/bin/python3.10 ~/catkin_ws/src/fr3_move/scripts/main.py