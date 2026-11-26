from itertools import chain

import numpy as np

from utils.constants import STEPS_PULL
from utils.kinematics import forward_kinematics
from utils.parse_urdf import JointConfig
from utils.trajectory import cal_road_points


def calculate_direct_line_move(q0:list[float],T_grasp,joints:list[JointConfig]):
    T_pre = forward_kinematics(joints,q0)
    # distance = np.linalg.norm(T_grasp[:3, 3] - T_pre[:3, 3])
    distance_single = (T_grasp[:3, 3] - T_pre[:3, 3]) / STEPS_PULL
    print(10*'===')
    print(f'distance_single:{distance_single}')
    array_road_points = []
    array_road_points = [q0]
    time_array = []
    time_array = [0]
    T_step = T_pre.copy()
    q = q0.copy()
    for _ in range(1,STEPS_PULL + 1):
        T_step[:3, 3] = T_step[:3, 3] + distance_single
        points,times = cal_road_points(q,T_step ,joints)
        print(f'\n========================= times:{times}')
        if points is None:
            return None,None
        # array_road_points.append(points[1:])
        # array_road_points = array_road_points + list(chain.from_iterable(points[1:]))
        array_road_points.extend(points[1:])
        # if len(time_array) == 0 :
        #     time_array.extend(times)
        # else:
        #     print(time_array)
        #     old_last_time = time_array[-1]
        #     time_array.extend([x + old_last_time for x in times[1:]])
        old_last_time = time_array[-1]
        time_array.extend([x + old_last_time for x in times[1:]])
        q = points[-1]
    return array_road_points,time_array