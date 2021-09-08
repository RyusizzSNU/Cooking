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

        rospy.Subscriber('/dope/%s/pose_pan_handle_handle'%name, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber('/dope/%s/pose_oil_bowl'%name, PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_carrot'%name, PoseStamped, self.carrot_callback)
        rospy.Subscriber('/dope/%s/pose_onion'%name, PoseStamped, self.onion_callback)
        rospy.Subscriber('/dope/%s/pose_knife_handle'%name, PoseStamped, self.knife_handle_callback)
        rospy.Subscriber('/dope/%s/pose_paddle_handle'%name, PoseStamped, self.paddle_handle_callback)
        rospy.Subscriber('/dope/%s/pose_rice_bowl'%name, PoseStamped, self.rice_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_salt_bowl'%name, PoseStamped, self.salt_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_board_handle'%name, PoseStamped, self.board_handle_callback)
        rospy.Subscriber('/dope/%s/pose_switch'%name, PoseStamped, self.switch_callback)
        rospy.Subscriber('/dope/%s/pose_spam'%name, PoseStamped, self.spam_callback)
        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback

    def board_handle_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['board_handle'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def carrot_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['carrot'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def onion_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['onion'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def pan_handle_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['pan_handle_handle'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def oil_bowl_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['oil_bowl'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def knife_handle_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['knife_handle'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def paddle_handle_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['paddle_handle'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def rice_bowl_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['rice_bowl'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def salt_bowl_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['salt_bowl'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def switch_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['switch'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def spam_callback(self, data):
        p, o = data.pose.position, data.pose.orientation
        self.obj_poses['spam'] = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def reset(self):
        self.obj_poses = {}

    def get_obj_pos(self, obj, interval=0.1, num_samples=10):
        while obj not in self.obj_poses:
            print(self.name, obj, 'not detected')
            time.sleep(1)

        poses = []
        for i in range(num_samples):
            poses.append(self.obj_poses[obj])
            time.sleep(interval)
        poses = np.array(poses)
        if len(poses) == 0:
            raise AssertionError
        std = np.std(poses, 0)

        return np.mean(poses, 0)