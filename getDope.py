import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class DopeReader(object):
    def __init__(self, name):
        self.objects = []
        self.obj_poses = {}
        self.name = name
        self.object_order = 0
        self.object_list = ['cracker', 'soup', 'meat']
        
        for i in self.object_list:
            rospy.Subscriber('/dope/{}/pose_{}'.format(name, i), PoseStamped, self.pos_callback)

    def pos_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        if len(self.object_list) <= self.object_order:
            pass	
        else:
            self.obj_poses[self.object_list[self.object_order]] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        self.object_order += 1

    def reset(self):
        self.obj_poses = {}
	self.object_order = 0

    def get_obj_pos(self, obj, interval=0.1, num_samples=10):
        while obj not in self.obj_poses:
            print(obj, 'not detected')
            time.sleep(1)

        poses = []
        for i in range(num_samples):
            poses.append(self.obj_poses[obj])
            time.sleep(interval)
        poses = np.array(poses)
        if len(poses) == 0:
            raise AssertionError
        std = np.std(poses, 0)
        # print(std)
        print(np.mean(poses,0))

        return np.mean(poses, 0)

rospy.init_node('test')
DR = DopeReader('R')
for i in DR.object_list:
    print(i)
    DR.get_obj_pos(i)

