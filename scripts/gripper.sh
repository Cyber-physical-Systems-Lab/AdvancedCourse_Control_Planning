#!/bin/bash
# 在容器（pulldrawer_dev 或 dobot_dev）里运行（start_sim.sh 要先启动好）：控制 2F-85 夹具开合
# 用法：
#   ./gripper.sh open      全开（finger_joint = 0.0）
#   ./gripper.sh close     全合（finger_joint = 0.725）
#   ./gripper.sh grip      夹箱子前壁用（finger_joint = 0.63，两指间距约 11.5 mm，前壁厚 10 mm）
#   ./gripper.sh 0.4       开合到指定角度（0.0 ~ 0.725 rad）

source /opt/ros/humble/setup.bash
[ -f /dobot_ws/install/setup.bash ] && source /dobot_ws/install/setup.bash   # 只有旧的 dobot_dev 容器里有

case "${1:-}" in
  open)  POS=0.0 ;;
  close) POS=0.725 ;;
  grip)  POS=0.63 ;;
  ''|*[!0-9.]*) echo "用法：./gripper.sh open | close | grip | <0.0~0.725>"; exit 1 ;;
  *)     POS="$1" ;;
esac

echo "夹具 finger_joint -> $POS"
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: $POS, max_effort: 10.0}}"
