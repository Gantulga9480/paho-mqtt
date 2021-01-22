import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from app.utils import BUFFER_THRESHOLD


class Kinect:

    def __init__(self, id_name, type_is=None):
        self.id_name = id_name
        self.type = type_is
        self.bridge = CvBridge()
        self.depth_buffer = list()
        self.rgb_buffer = list()
        self.is_streaming = False
        self.k1_depth_ready = False
        self.k1_rgb_ready = False

        rospy.init_node(f'kinect_{id_name}_node', anonymous=True)
        if type_is is 'xbox':
            rospy.Subscriber('/camera/depth/image_raw', Image, self.callback1)
            rospy.Subscriber('/camera/rgb/image_color', Image, self.callback2)
        elif type_is is 'azure':
            rospy.Subscriber('/depth/image_raw', Image, self.callback1)
            rospy.Subscriber('/rgb/image_color', Image, self.callback2)
        # rospy.spin()

    def is_ready(self):
        if self.k1_depth_ready and self.k1_rgb_ready:
            return True
        else:
            return False

    def callback1(self, msg):
        """Depth data from kinect"""
        self.k1_depth_ready = True
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        img = img[..., None].repeat(3, -1).astype("uint8")
        if self.is_streaming:
            self.depth_buffer.append(np.round(img).astype(np.uint8))
        if self.depth_buffer.__len__() > BUFFER_THRESHOLD:
            print("Depth Buffer full")

    def callback2(self, msg):
        """RGB data from kinect"""
        self.k1_rgb_ready = True
        img = self.bridge.imgmsg_to_cv2(msg)
        if self.is_streaming:
            self.rgb_buffer.append(np.round(img).astype(np.uint8))
        if self.rgb_buffer.__len__() > BUFFER_THRESHOLD:
            print("RGB Buffer full")

    def img_show(self, img):
        cv2.imshow('xbox_kinext_depth', img)
        cv2.waitKey(1)
