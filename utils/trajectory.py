from pydoc import cli

import numpy as np

import rclpy
from rclpy.action.client import ClientGoalHandle
from utils.constants import HOME_NOVA5, MAX_JOINT_SPEED, NUM_ROAD_POINT, STEP_LIMIT, WAYPOINT_MAX_STEP
from utils.kinematics import calculate_IK, forward_kinematics
from utils.parse_urdf import parse_urdf
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration

def send_trajectory(node,names,way_points,times):
    client = ActionClient(node, FollowJointTrajectory,
                      '/nova5_group_controller/follow_joint_trajectory')
    if not client.wait_for_server(10):
        print('Can not find the server')
        return
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    for way_point,t in zip(way_points[1:],times[1:]):
        point = JointTrajectoryPoint()
        point.positions = way_point.tolist()
        point.time_from_start = Duration(seconds=t).to_msg()
        goal.trajectory.points.append(point)
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node,send_future)
    goal_handle:ClientGoalHandle = send_future.result()
    if not goal_handle.accepted:
        print('rejected')
    else:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result().result
        print(result.error_code, result.error_string)
        
def cal_road_points(start:list[float],T_target: np.ndarray, joints):
    # joints = parse_urdf()
    q0 = np.array(start, dtype=float)
    q_target,status = calculate_IK(T_target,q0,joints)
    if not status:
        return None,None
    max_delta_angle = np.max(np.abs(q_target - q0))
    max_road_points = NUM_ROAD_POINT
    while max_delta_angle / max_road_points > WAYPOINT_MAX_STEP:
        max_road_points = max_road_points + 1
    step_delta_angle = (q_target - q0) / (max_road_points)
    array_road_points = []
    array_road_points.append(q0)
    max_time = max_delta_angle / MAX_JOINT_SPEED
    time_gap = max_time / max_road_points
    time_array = []
    time_array.append(0)
    for _ in range(1,max_road_points + 1):
        last_point = array_road_points[len(array_road_points) - 1]
        array_road_points.append(last_point + step_delta_angle)
        last_time = time_array[len(time_array) - 1]
        time_array.append(last_time + time_gap)
    return array_road_points,time_array