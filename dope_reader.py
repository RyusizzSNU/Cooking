import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from scipy.spatial.transform import Rotation as R

class DopeReader(object):
    def __init__(self, name):
        self.objects = []
        self.obj_poses = {}
        self.name = name

        rospy.Subscriber('/dope/%s/pose_pan_handle_handle'%name, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber('/dope/%s/pose_oil_bowl'%name, PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_carrot'%name, PoseStamped, self.carrot_callback)
        rospy.Subscriber('/dope/%s/pose_knife_handle'%name, PoseStamped, self.knife_handle_callback)
        rospy.Subscriber('/dope/%s/pose_paddle_handle'%name, PoseStamped, self.paddle_handle_callback)
        rospy.Subscriber('/dope/%s/pose_rice_bowl'%name, PoseStamped, self.rice_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_salt_bowl'%name, PoseStamped, self.salt_bowl_callback)
        rospy.Subscriber('/dope/%s/pose_board_handle'%name, PoseStamped, self.board_handle_callback)
        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback

    def board_handle_callback(self, data):
        self.obj_poses['board_handle'] = data.pose

    def carrot_callback(self, data):
        self.obj_poses['carrot'] = data.pose

    def pan_handle_callback(self, data):
        self.obj_poses['pan_handle_handle'] = data.pose

    def oil_bowl_callback(self, data):
        self.obj_poses['oil_bowl'] = data.pose

    def knife_handle_callback(self, data):
        self.obj_poses['knife_handle'] = data.pose

    def paddle_handle_callback(self, data):
        self.obj_poses['paddle_handle'] = data.pose

    def rice_bowl_callback(self, data):
        self.obj_poses['rice_bowl'] = data.pose

    def salt_bowl_callback(self, data):
        self.obj_poses['salt_bowl'] = data.pose

    def get_obj_pos(self, obj):
        if not obj in self.obj_poses:
            return None
        return self.obj_poses[obj]