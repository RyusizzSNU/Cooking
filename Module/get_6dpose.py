import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image



class DopeController(object):
    def __init__(self):
        #self.bridge = CvBridge()
        self.pan_pose = None
        pan_handle_topic = '/dope/pose_pan_handle_handle'
        oil_bowl_topic = '/dope/pose_oil_bowl'
        knife_handle_topic = '/dope/pose_knife_handle'
        board_handle_topic = '/dope/pose_board_handle'
        switch_topic = '/dope/pose_switch'

        rospy.Subscriber(pan_handle_topic, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber(oil_bowl_topic, PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber(knife_handle_topic, PoseStamped, self.knife_handlel_callback)
        rospy.Subscriber(board_handle_topic, PoseStamped, self.board_handle_callback)
        rospy.Subscriber(switch_topic, PoseStamped, self.switch_callback)

        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback)

    def pan_handle_callback(self, data):
        self.pan_handle_pose = data.pose

    def oil_bowl_callback(self, data):
        self.oil_bowl_pose = data.pose

    def knife_handlel_callback(self, data):
        self.knife_handle_pose = data.pose

    def board_handle_callback(self, data):
        self.board_handle_pose = data.pose

    def switch_callback(self, data):
        self.switch_pose = data.pose

    def salt_bowl_callback(self, data):
        self.salt_bowl_pose = data.pose

    def rice_bowl_callback(self, data):
        self.rice_bowl_pose = data.pose


    def get_pose(self, object_name):
        try:
            if object_name == 'pan_handle':
                return self.pan_handle_pose.position, self.pan_handle_pose.orientation
            elif object_name == 'oil_bowl':
                return self.oil_bowl_pose.position, self.oil_bowl_pose.orientation
            elif object_name == 'knife_handle':
                return self.knife_handle_pose.position, self.knife_handle_pose.orientation
            elif object_name == 'board_handle':
                return self.board_handle_pose.position, self.board_handle_pose.orientation
            elif object_name == 'switch':
                return self.switch_pose.position, self.switch_pose.orientation
            elif object_name == 'salt_bowl':
                return self.salt_bowl_pose.position, self.salt_bowl_pose.orientation
            elif object_name == 'rice_bowl':
                return self.rice_bowl_pose.position, self.rice_bowl_pose.orientation
        except Exception as e:
            print(e)
            print('[ERROR] Object is not detected yet!!!')
            return None, None

if __name__ == '__main__':
    rospy.init_node('dope_get_6dpose')
    dope_confroller = DopeController()
    rospy.sleep(3)
    pan_pose, pan_orient = dope_confroller.get_pose('pan_handle')
    print('pan_pose x, y, z', pan_pose.x, pan_pose.y, pan_pose.z)
    print('pan_orient x, y, z, w', pan_orient.x, pan_orient.y, pan_orient.z, pan_orient.w)


    oil_bowl_pose, oil_bowl_orient = dope_confroller.get_pose('oil_bowl')
    print('oil_bowl_pose x, y, z', oil_bowl_pose.x, oil_bowl_pose.y, oil_bowl_pose.z)
    print('oil_bowl_orient x, y, z, w', oil_bowl_orient.x, oil_bowl_orient.y, oil_bowl_orient.z, oil_bowl_orient.w)