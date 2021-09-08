import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import matplotlib.pyplot as plt

class SegmentationController(object):
    def __init__(self, thres, borders):
        self.bridge = CvBridge()
        depth_img_topic = '/cam_R/depth/image_rect_raw'
        rospy.Subscriber(depth_img_topic, Image, self.find_corners)
        self.p = rospy.Publisher("segmentation", Image, queue_size=1)
        self.thres = thres
        self.borders = borders
        self.corners = None
        self.active = False

    def find_corners(self, msg):
        if self.active:
            self.img_msg = self.bridge.imgmsg_to_cv2(msg)
            # self.img_msg = (self.img_msg/np.max(self.img_msg)*255).astype(np.uint8)
            # img = self.img_msg[self.borders[0][0]:self.borders[1][0],self.borders[0][1]:self.borders[1][1]]
            # img = cv2.adaptiveThreshold(img, np.max(img), cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
            # ret, thr = cv2.threshold(self.img_msg, 637, 255, cv2.THRESH_OTSU)
            # img, contours, hierarchy = cv2.findContours(thr, 1, 2)
            np_img = np.asanyarray(self.img_msg)
            np_img_filtered = np.zeros_like(np_img).astype(np.uint8)
            np_img_filtered[np_img < self.thres] = 255
            img_patch = np_img_filtered[self.borders[0][1]:self.borders[1][1],self.borders[0][0]:self.borders[1][0]]
            ret, img_patch = cv2.threshold(img_patch, 125, 255, cv2.THRESH_OTSU)
            # cv2.cvtColor(np_img_filtered, cv2.COLOR_BGR2GRAY)
            self.img_patch = img_patch

            contours, hierarchy = cv2.findContours(img_patch, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # print(contours[0])
            if len(contours) > 0:

                cnt = contours[-1]
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # img_patch = cv2.cvtColor(img_patch, cv2.COLOR_GRAY2BGR)
                # image = cv2.drawContours(img_patch, contours, -1, (0, 255, 0), 3)
                # image = cv2.drawContours(img_patch, [box], 0, (0,0,255), 2)
                self.corners = self.add_borders(box)

                np_img_filtered = cv2.cvtColor(np_img_filtered, cv2.COLOR_GRAY2BGR)
                for cnt in contours:
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    corners = self.add_borders(box)
                    np_img_filtered = cv2.drawContours(np_img_filtered, [corners], 0, (0,0,255), 2)

                self.image_boxed = np_img_filtered
                img_msg = CvBridge().cv2_to_imgmsg(self.image_boxed, "bgr8")
                img_msg.header.frame_id = 'segmentation'
                img_msg.header.stamp = rospy.Time.now()
                self.p.publish(img_msg)
                # self.active = False

    def stimulate(self, active):
        self.active = active

    def add_borders(self, box):
        # box_comp = []
        # for b in box:
        #     box_comp.append(b + self.borders[0])
        # return box_comp

        return box + self.borders[0]

    def get_corners(self):
        try:
            return self.corners
        except Exception as e:
            print(e)
            return None

    def get_image(self):
        try:
            return self.image_boxed
        except Exception as e:
            print(e)
            return None


    # def depth_camera_cb(self, msg):
    #     print("msg", msg)
    #     self.img_msg = self.bridge.imgmsg_to_cv2(msg)
    #     np_img = np.asanyarray(self.img_msg)
    #     show_mode = True
    #     if show_mode:
    #         print(np_img.shape)
    #         # Show images
    #         cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)
    #         cv2.imshow('RealSense_depth', np_img)
    #         cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('depth_seg')
    seg_controller = SegmentationController(635, [(75,220), (240,340)]) #635
    # seg_controller = SegmentationController(635, [(240, 80), (320, 280)])  # 635
    seg_controller.stimulate()
    rospy.sleep(100)

    # cv2.namedWindow('Depth_Segmentation', cv2.WINDOW_AUTOSIZE)
    # while True:
    corners = seg_controller.get_corners()
    image = seg_controller.get_image()
    if corners is not None:
        # cv2.imshow('Depth_Segmentation', seg_controller.img_output)
        # cv2.waitKey(1)
        print(corners)
        # plt.imshow(image)
        plt.imshow(seg_controller.img_patch)
        plt.show()
    # rospy.spin()
