#!/bin/bash
# 在容器 dobot_dev 里运行（Gazebo 要先启动好）：把 Nova5 移到指定姿态
# 用法：
#   ./go_home.sh          回 home  [0, 0.5378, -1.1869, 0, 1.7785, 0]，用时 5 秒
#   ./go_home.sh zero     回零位   [0, 0, 0, 0, 0, 0]（Gazebo 刚启动时的姿态）
#   ./go_home.sh home 8   回 home，用时 8 秒（第二个参数是秒数）

source /opt/ros/humble/setup.bash
source /dobot_ws/install/setup.bash

TARGET="${1:-home}"
SECONDS_TO_MOVE="${2:-5}"

case "$TARGET" in
  home) POSITIONS="[0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0]" ;;
  zero) POSITIONS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" ;;
  *)    echo "不认识的目标: $TARGET（只能是 home 或 zero）"; exit 1 ;;
esac

echo "移动到 $TARGET: $POSITIONS，用时 ${SECONDS_TO_MOVE} 秒"

ros2 action send_goal /nova5_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: $POSITIONS, time_from_start: {sec: $SECONDS_TO_MOVE, nanosec: 0}}]}}"
