import stat

import numpy as np

from rclpy import node
from utils.approach import start_subscribe_joints
from utils.constants import HOME_NOVA5
from utils.kinematics import calculate_IK, calculate_Jocabin, forward_kinematics
from utils.parse_urdf import parse_urdf
from utils.trajectory import cal_road_points, send_trajectory
from utils.action_pull import calculate_direct_line_move


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    qs = [0.3, 0.2, -0.8, 0.1, 1.0, -0.2]
    joints = parse_urdf()
    T = forward_kinematics(joints,qs)
    # print(T)
    # print(50*'*')
    joints = parse_urdf()
    J = calculate_Jocabin([0, 0, 0, 0, 0, 0], joints)
    # print(J)
    # print(50*'*')
    J = calculate_Jocabin([0.3, 0.2, -0.8, 0.1, 1.0, -0.2], joints)
    # print(J)
    # print(50*'*IK')
    q_now,status = calculate_IK(forward_kinematics(joints,[0.3, 0.2, -0.8, 0.1, 1.0, -0.2]),HOME_NOVA5,joints)
    # print(f'status={status}\n')
    # print(q_now)
    # print(50*'*IK2')
    q_now,status = calculate_IK(forward_kinematics(joints,[-0.5, 0.8, -1.5, 0.4, 1.2, 0.7]),HOME_NOVA5,joints)
    # print(f'status={status}\n')
    # print(q_now)
    # print(50*'*Trajectory')
    T_target = forward_kinematics(joints, [-1.2, 0.9, -1.6, 0.5, 1.0, 0.8])
    points,times = cal_road_points(HOME_NOVA5, T_target, joints)
    # print(10*'===')
    # print(len(points))
    # print(10*'===')
    # print(times)
    # node,q = start_subscribe_joints()
    # print(q)
    print(10*'===')
    node,q = start_subscribe_joints()
    print('start to cal down action')
    T_target_pre_grasp = forward_kinematics(joints, [-1.2, 0.9, -1.6, 0.5, 1.0, 0.8])
    T_target_pre_grasp = np.array([[1,  0,  0,  0.574],
                  [0, -1,  0, -0.2  ],
                  [0,  0, -1,  0.654],
                  [0,  0,  0,  1    ]])
    points, times = cal_road_points(q, T_target_pre_grasp, joints)
    send_trajectory(node, ['joint1','joint2','joint3','joint4','joint5', 'joint6'], points, times)
    # print(points)
    # print(times)
    T_grasp = T_target_pre_grasp.copy()
    T_grasp[2,3] = T_target_pre_grasp[2,3] - 0.1
    print(f'T_pre:{T_target_pre_grasp}')
    print(f'T_grasp:{T_grasp}')
    print('start to cal down action')
    down_action_way_points, down_action_times = calculate_direct_line_move(points[-1],T_grasp,joints)
    # print(down_action_way_points)
    # print(down_action_times)
    send_trajectory(node, ['joint1','joint2','joint3','joint4','joint5', 'joint6'], down_action_way_points, down_action_times)
    # calculate the horizontal movement
    T_pull_horizontal_destination = T_grasp.copy()
    T_pull_horizontal_destination[0, 3] -= 0.16
    print('start to cal horizontal movement action')
    pull_action_way_points, pull_action_times = calculate_direct_line_move(down_action_way_points[-1],T_pull_horizontal_destination,joints)
    # print(down_action_way_points)
    # print(down_action_times)
    send_trajectory(node, ['joint1','joint2','joint3','joint4','joint5', 'joint6'], pull_action_way_points, pull_action_times)


    
