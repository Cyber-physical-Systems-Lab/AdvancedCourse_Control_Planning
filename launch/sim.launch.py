"""Gazebo Classic: Nova5 + Robotiq 2F-85 with ros2_control.

Based on Dobot's dobot_gazebo/launch/gazebo_moveit.launch.py; only the robot model changed
and a gripper controller was added.

Run (inside the dobot_dev container):
  ros2 launch /dobot_ws/shared/ros_ws/PullDrawerProject/launch/sim.launch.py              # robot + shelf
  ros2 launch /dobot_ws/shared/ros_ws/PullDrawerProject/launch/sim.launch.py shelf:=false # robot only

Provides:
  /nova5_group_controller/follow_joint_trajectory   arm  (control_msgs/action/FollowJointTrajectory)
  /gripper_controller/gripper_cmd                    gripper (control_msgs/action/GripperCommand)
  /joint_states                                      joint1..joint6, finger_joint and the mimic joints
  /shelf/joint_states                                drawer_0..2 (how far each bin is pulled out, m)
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESC_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'description'))
URDF_FILE = os.path.join(DESC_DIR, 'nova5_2f85_sim.urdf')
SHELF_FILE = os.path.join(DESC_DIR, 'shelf.sdf')
ENTITY_NAME = 'nova5_2f85'


def load_controller(name):
    return ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', name],
        output='screen')


def generate_launch_description():
    with open(URDF_FILE, 'r') as f:
        robot_description = f.read().replace('__DESC_DIR__', DESC_DIR)
    # spawn_entity.py parses this string with lxml, which rejects an <?xml ... encoding=...?> line
    robot_description = re.sub(r'^\s*<\?xml[^>]*\?>', '', robot_description)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', ENTITY_NAME],
        output='screen')

    spawn_shelf = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', SHELF_FILE, '-entity', 'shelf'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('shelf')))

    load_jsb = load_controller('joint_state_broadcaster')
    load_arm = load_controller('nova5_group_controller')
    load_gripper = load_controller('gripper_controller')

    # same order as Dobot's launch: spawn robot -> broadcaster -> arm -> gripper
    return LaunchDescription([
        DeclareLaunchArgument('shelf', default_value='true',
                              description='spawn the shelf with bins (description/shelf.sdf)'),
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm, on_exit=[load_gripper])),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_shelf,
    ])
