#!/bin/bash
# 在容器 dobot_dev 里运行：启动 Gazebo + Nova5（ros2_control 控制器 + MoveIt）
# 用法：./start_gazebo.sh        停止：在这个终端按 Ctrl+C

source /opt/ros/humble/setup.bash
source /dobot_ws/install/setup.bash
export DOBOT_TYPE=nova5

exec ros2 launch dobot_gazebo gazebo_moveit.launch.py
