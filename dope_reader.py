import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from scipy.spatial.transform import Rotation as R

class DopeReader(object):
    def __init__(self):
        self.objects = []
        self.obj_poses = {}

        rospy.Subscriber('/dope/pose_pan_handle_handle', PoseStamped, self.pan_handle_callback)
        rospy.Subscriber('/dope/pose_oil_bowl', PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber('/dope/pose_carrot', PoseStamped, self.carrot_callback)
        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback

    def carrot_callback(self, data):
        self.obj_poses['carrot'] = data.pose

    def pan_handle_callback(self, data):
        self.obj_poses['pan_handle_handle'] = data.pose

    def oil_bowl_callback(self, data):
        self.obj_poses['oil_bowl'] = data.pose

    def get_obj_pos(self, obj):
        if not obj in self.obj_poses:
            return None
        return self.obj_poses[obj]