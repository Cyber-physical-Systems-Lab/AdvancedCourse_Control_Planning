import xml.etree.ElementTree as ET

class Limit():
    def __init__(self):
        self.effort = 0.0
        self.lower = 0.0
        self.upper = 0.0
        self.velocity = 0.0
    def __repr__(self):
        return f"Limit(lower={self.lower}, upper={self.upper}, effort={self.effort}, velocity={self.velocity})"


class JointConfig():
    def __init__(self):
        self.joint_type = ''
        self.joint_name = ''
        self.joint_rpy = []
        self.joint_xyz = []
        self.joint_axis = []
        self.limit = Limit()
    def __repr__(self):
        return (f"{self.joint_name} ({self.joint_type}): "
                f"xyz={self.joint_xyz}, rpy={self.joint_rpy}, axis={self.joint_axis}, {self.limit}")

def parse_urdf():
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    tree = ET.parse('./assets/nova5_gazebo.urdf')
    root  = tree.getroot()
    joints = []
    for name in JOINT_NAMES:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None:
            raise ValueError(f"Cant find {name} in the URDF")
        joint_config = JointConfig()
        joint_rpy = joint.find('origin').get('rpy')
        joint_xyz = joint.find('origin').get('xyz')
        # joint_parent_link = joint.find('parent').get('link')
        # joint_child_link = joint.find('child').get('link')
        joint_xis = joint.find('axis').get('xyz')
        joint_config.joint_type = joint.get('type')
        joint_config.joint_name = joint.get('name')
        joint_config.joint_rpy = [float(v) for v in joint_rpy.split()] 
        joint_config.joint_xyz = [float(v) for v in joint_xyz.split()]
        joint_config.joint_axis = [float(v) for v in joint_xis.split()]
        joint_config.limit.effort = float(joint.find('limit').get('effort'))
        joint_config.limit.lower = float(joint.find('limit').get('lower'))
        joint_config.limit.upper = float(joint.find('limit').get('upper'))
        joint_config.limit.velocity = float(joint.find('limit').get('velocity'))
        joints.append(joint_config)
    return joints
if __name__ == '__main__':
    joints = parse_urdf()
    print(joints)