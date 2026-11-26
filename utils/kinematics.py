import dis
import math
from shlex import join
from scipy.spatial.transform import Rotation
import numpy as np
from utils.constants import DEFAULT_DELTA_ANGLE, DLS_DAMPING, HOME_NOVA5, IK_MAX_ITER, IK_TOL_POS, IK_TOL_ROT, MAX_JOINT_SPEED, NUM_ROAD_POINT, STEP_LIMIT, STEPS_PULL, THRESHOLD_RPY, WAYPOINT_MAX_STEP
from utils.parse_urdf import JointConfig, parse_urdf

def RPY_R(joint_config:JointConfig):
    r = joint_config.joint_rpy[0]
    c = np.cos(r)
    s= np.sin(r)
    T = np.array(
        [
            [1,0,0],
            [0,c,-s],
            [0,s,c]
        ]
    )
    return T


def RPY_P(joint_config:JointConfig):
    r = joint_config.joint_rpy[1]
    c = np.cos(r)
    s= np.sin(r)
    T = np.array(
        [
            [c,0,s],
            [0,1,0],
            [-s,0,c]
        ]
    )
    return T


def RPY_Y(joint_config:JointConfig):
    r = joint_config.joint_rpy[2]
    c = np.cos(r)
    s= np.sin(r)
    T = np.array(
        [
            [c,-s,0],
            [s,c,0],
            [0,0,1]
        ]
    )
    return T

def get_joint_transform(joint_config:JointConfig,q:float):
    xyz = []
    xyz = joint_config.joint_xyz
    rpy = []
    rpy = joint_config.joint_rpy
    if abs(joint_config.joint_axis[0]) > THRESHOLD_RPY or abs(joint_config.joint_axis[1]) > THRESHOLD_RPY:
        raise ValueError(f"Joint的axis不是Z轴了 需要重新按照绕着任意单元轴来重新计算 目前是按照默认的Z或者-Z轴来计算的")
    if(joint_config.joint_axis[2] < 0):
        q = -q
    T = np.identity(4)
    x_matrix = RPY_R(joint_config)
    y_matrix = RPY_P(joint_config)
    z_matrix = RPY_Y(joint_config)
    T_rpy = z_matrix @ y_matrix @ x_matrix
    c = np.cos(q)
    s = np.sin(q)
    rotation_matrxix = np.array(
        [
            [c,-s,0],
            [s,c,0],
            [0,0,1]
        ]
    )
    T[:3,:3] = T_rpy
    T[:3,3] = xyz
    axis_homo_matrix = np.identity(4)
    axis_homo_matrix[:3,:3] = rotation_matrxix
    # axis_homo_matrix[:3,3] = [0,0,0,1]
    return T @ axis_homo_matrix

def forward_kinematics(joints:list[JointConfig], qs:list[float]):
    T = np.identity(4)
    for joint, q in zip(joints, qs):
        T = T @ get_joint_transform(joint,q)
    return T

def calculate_Jocabin(qs:list[float],joints:list[JointConfig]):
    T = forward_kinematics(joints,qs)
    jocobin = np.identity(6)
    for i in range(len(qs)):
        qq = qs.copy()
        qq[i] = qq[i] + DEFAULT_DELTA_ANGLE
        TT = forward_kinematics(joints,qq)
        jocobin[:3,i] = (TT[:3, 3] - T[:3, 3]) / DEFAULT_DELTA_ANGLE
        R = T[:3,:3]
        RR = TT[:3,:3]
        delta_R = RR @ R.T
        rotvec = Rotation.from_matrix(delta_R).as_rotvec()
        jocobin[3:,i] = rotvec / DEFAULT_DELTA_ANGLE
    return jocobin

def calculate_IK(T_target, q0:list[float], joints:list[JointConfig]):
    q_now = np.array(q0, dtype=float)
    lower = []
    upper = []
    for joint in joints:
        lower.append(joint.limit.lower)
        upper.append(joint.limit.upper)
    for index in range(IK_MAX_ITER):
        print(f'第{index}次迭代')
        T_current = forward_kinematics(joints, q_now)
        error_xyz = T_target[:3,3] - T_current[:3,3]
        error_rotataion_matrix = T_target[:3,:3] @ T_current[:3,:3].T
        error_rot = Rotation.from_matrix(error_rotataion_matrix).as_rotvec()
        error = np.concatenate([error_xyz, error_rot]) 
        if np.linalg.norm(error[:3]) < IK_TOL_POS and np.linalg.norm(error[3:]) < IK_TOL_ROT:
            return q_now, True
        jacobin = calculate_Jocabin(q_now, joints)
        delta_q = jacobin.T @ np.linalg.solve(jacobin @ jacobin.T + DLS_DAMPING**2 * np.identity(6),error)
        max_step = np.max(np.abs(delta_q))
        if max_step > STEP_LIMIT:
            delta_q = delta_q * (STEP_LIMIT / max_step)
        q_now = q_now + delta_q
        q_now = np.clip(q_now, lower, upper)
    return q_now, False
        

if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    qs = [0, 0, 0, 0, 0, 0]
    joints = parse_urdf()
    T = forward_kinematics(joints,qs)
    print(T)