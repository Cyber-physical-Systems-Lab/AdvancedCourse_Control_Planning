#!/bin/bash
# 在容器里运行（start_sim.sh 要先启动好）：把场景复位，方便重新拉一次
#   1. 夹具张开
#   2. 删掉架子（先删，手臂回 home 时就不会刮到箱子）
#   3. 手臂回 home
#   4. 重新生成架子（箱子回到初始位置）
#   5. 打印箱子的状态，确认 drawer_i 和 bin_i_tilt 都接近 0
# 用法：
#   ./reset_scene.sh

source /opt/ros/humble/setup.bash
[ -f /dobot_ws/install/setup.bash ] && source /dobot_ws/install/setup.bash   # 只有旧的 dobot_dev 容器里有

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SHELF_FILE="$SCRIPT_DIR/../description/shelf.sdf"

if [ ! -f "$SHELF_FILE" ]; then
  echo "找不到架子文件: $SHELF_FILE"; exit 1
fi

echo "===== 1/5 夹具张开 ====="
"$SCRIPT_DIR/gripper.sh" open

echo "===== 2/5 删掉架子 ====="
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'shelf'}"
sleep 1

echo "===== 3/5 手臂回 home ====="
"$SCRIPT_DIR/go_home.sh" home 5

echo "===== 4/5 重新生成架子 ====="
ros2 run gazebo_ros spawn_entity.py -file "$SHELF_FILE" -entity shelf
sleep 2

echo "===== 5/5 箱子状态（drawer_i、bin_i_tilt 都应该接近 0）====="
timeout 5 ros2 topic echo /shelf/joint_states --once \
  || echo "5 秒内没收到 /shelf/joint_states，看一下 Gazebo 里架子有没有出来"
