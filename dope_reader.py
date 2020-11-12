import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from scipy.spatial.transform import Rotation as R

class DopeReader(object):
    def __init__(self, name):
        self.objects = []
        self.obj_poses = {}
        self.name = name

        rospy.Subscriber('/dope/pose_pan_handle_handle', PoseStamped, self.pan_handle_callback)
        rospy.Subscriber('/dope/pose_oil_bowl', PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber('/dope/pose_carrot', PoseStamped, self.carrot_callback)
        rospy.Subscriber('/dope/pose_knife_handle', PoseStamped, self.knife_handle_callback)
        rospy.Subscriber('/dope/pose_paddle_handle', PoseStamped, self.paddle_handle_callback)

        rospy.Subscriber('/dope/pose_rice_bowl', PoseStamped, self.rice_bowl_callback)
        rospy.Subscriber('/dope/pose_salt_bowl', PoseStamped, self.salt_bowl_callback)
        rospy.Subscriber('/dope/pose_board_handle', PoseStamped, self.board_handle_callback)
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