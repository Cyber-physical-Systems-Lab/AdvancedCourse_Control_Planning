#!/bin/bash
# 在容器（pulldrawer_dev 或 dobot_dev）里运行：启动 Gazebo + Nova5 + Robotiq 2F-85 夹具
# 用法：./start_sim.sh        停止：在这个终端按 Ctrl+C

source /opt/ros/humble/setup.bash
[ -f /dobot_ws/install/setup.bash ] && source /dobot_ws/install/setup.bash   # 只有旧的 dobot_dev 容器里有

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$SCRIPT_DIR/../launch/sim.launch.py"
