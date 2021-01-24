import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
import time
from pyk4a import PyK4A
from sensor_msgs.msg import Image
from app.utils import BUFFER_THRESHOLD


class Kinect:

    def __init__(self, id_name, type_is=None):
        self.id_name = id_name
        self.type = type_is
        self.bridge = CvBridge()
        self.depth_buffer = list()
        self.rgb_buffer = list()
        self.azure_rgb_buffer = list()
        self.azure_depth_buffer = list()
        self.is_streaming = False
        self.k_depth_ready = False
        self.k_rgb_ready = False
        self.a_depth_ready = False
        self.a_rgb_ready = False
        self.k4a = PyK4A()

        rospy.init_node(f'{id_name}_node', anonymous=True)
        rospy.Subscriber('/camera/depth/image', Image, self.callback1)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.callback2)
        print("kinect start")
        time.sleep(5)
        self.k4a.start()
        print("azure start")

    def is_ready(self):
        if self.k_depth_ready and self.k_rgb_ready:
            return True
        else:
            return False

    def callback1(self, msg):
        """Depth data from kinect"""
        # print("k1 depth")
        img_x = self.bridge.imgmsg_to_cv2(msg)
        cv2.normalize(img_x, img_x, 0, 255, cv2.NORM_MINMAX)
        img_x = np.round(img_x).astype(np.uint8)
        img = self.k4a.get_capture()
        img = img.depth
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        img = np.round(img).astype(np.uint8)
        self.k_depth_ready = True
        if self.is_streaming:
            self.depth_buffer.append(img_x)
            self.azure_depth_buffer.append(img)

    def callback2(self, msg):
        """RGB data from kinect"""
        # print(self.k_rgb_ready, self.k_depth_ready, self.a_rgb_ready, self.a_depth_ready)
        img_x = self.bridge.imgmsg_to_cv2(msg)
        img_x = np.round(img_x).astype(np.uint8)
        img = self.k4a.get_capture()
        img = img.color
        img = np.round(img).astype(np.uint8)
        self.k_rgb_ready = True
        if self.is_streaming:
            self.rgb_buffer.append(img_x)
            self.azure_rgb_buffer.append(img)

    def azure_color(self, msg):
        """RGB data from azure"""
        # print("azure rgb")
        img = self.bridge.imgmsg_to_cv2(msg)
        img = np.round(img).astype(np.uint8)
        self.a_rgb_ready = True
        if self.is_streaming:
            self.azure_rgb_buffer.append(img)

    def azure_depth(self, msg):
        """Depth data from azure"""
        # print("azure depth")
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        img = np.round(img).astype(np.uint8)
        self.a_depth_ready = True
        if self.is_streaming:
            self.azure_depth_buffer.append(img)
