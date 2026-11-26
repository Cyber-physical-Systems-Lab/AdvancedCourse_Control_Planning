import rclpy                      
from rclpy.action import ActionClient
from rclpy.node   import Node     
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

class ApproachNode(Node):
    def __init__(self):
        super().__init__('Subscriber_JointState_Node')
        self.q_current = None
        self.actionClient = ActionClient(self,FollowJointTrajectory,'/nova5_group_controller/follow_joint_trajectory')
        self.sub = self.create_subscription(JointState,'/joint_states',self.joint_listener_callback,10)

    def joint_listener_callback(self,message:JointState):
        print(message)
        self.q_current = []
        # joints_angle = []
        for index in range(1,7):
            print(message)
            i = message.name.index(f'joint{index}')
            self.q_current.append(message.position[i])
    def get_current_q(self):
        return self.q_current

def start_subscribe_joints():
    rclpy.init()
    node = ApproachNode()
    while node.get_current_q() is None:
        rclpy.spin_once(node)
    return node,node.get_current_q()