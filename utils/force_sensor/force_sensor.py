import numpy as np

from geometry_msgs.msg import WrenchStamped
from rclpy.node   import Node

import rclpy
from utils.constants import CONTACT_CONSECUTIVE_SAMPLES, CONTACT_STD_MULTIPLIER, NUM_DORCE_DATA_SAVE, NUM_FORCE_DATA_DROP

class ForceDataNode():
    def __init__(self,node):
        self.switch = False
        self.monitor = False
        self.contact = False
        # super().__init__('Subscriber_Force_Subscribetion_Node')
        self.node = node
        self.sub = self.node.create_subscription(WrenchStamped,'/wrist_ft/wrench',self.__callback_force,10)
        self.force_datas = []
    def __callback_force(self,data:WrenchStamped):
        if(self.switch):
            local_data = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
            self.force_datas.append(local_data)
            # print(f'ForceDataNode:{data}')
        elif(self.monitor):
            print('********************MONITOR*************************')
            tare_fx = data.wrench.force.x - self.zero_point_fx
            print(f'tare_fx = {tare_fx}')
            if(tare_fx > CONTACT_STD_MULTIPLIER * self.std_fx):
                self.number_over_std = self.number_over_std + 1
                if(self.number_over_std >= CONTACT_CONSECUTIVE_SAMPLES):
                    self.monitor = False
                    self.contact = True
                    print('********************touched the box*************************')
                    print(f'{data.header.stamp.sec + data.header.stamp.nanosec * 1e-9}')
                    print(f'{tare_fx}')
            else:
                self.number_over_std = 0
    def get_zero_points_std(self):
        self.force_datas.clear()
        self.switch = True
        while len(self.force_datas) < (NUM_FORCE_DATA_DROP + NUM_DORCE_DATA_SAVE):
            rclpy.spin_once(self.node)
        self.switch = False
        print(self.force_datas[NUM_FORCE_DATA_DROP:])
        np_force_data = np.array(self.force_datas[NUM_FORCE_DATA_DROP:])
        mean,std = np_force_data.mean(axis=0),np_force_data.std(axis=0)
        print(f'mean,std:{mean,std}')
        return mean,std
    def start_monitoring(self,mean_fx,std_fx):
        self.zero_point_fx = mean_fx
        self.std_fx = std_fx
        self.monitor = True
        self.contact = False
        self.number_over_std = 0
        print(f'MONITOR zero_point_fx = {self.zero_point_fx},std_fx = {self.std_fx}, CONTACT_STD_MULTIPLIER * std_fx = {CONTACT_STD_MULTIPLIER * std_fx}')
        
    
    