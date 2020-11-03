import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class RealsenseController(object):
    def __init__(self):
        self.bridge = CvBridge()
        depth_img_topic = '/camera/depth/image_rect_raw'
        rospy.Subscriber(depth_img_topic, Image, self.depth_camera_cb)

    def head_camera_cb(self, msg):
        self.img_msg = self.bridge.imgmsg_to_cv2(msg)
        np_img = np.asanyarray(self.img_msg)

        # Show images
        cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense_depth', np_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('get_depth_img')
    realsensecontroller = RealsenseController()

    rospy.spin()
