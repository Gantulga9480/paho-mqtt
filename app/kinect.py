import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
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
        self.is_streaming = False
        self.k_depth_ready = False
        self.k_rgb_ready = False

        if type_is is 'xbox':
            rospy.init_node(f'{id_name}_node', anonymous=True)
            pass
        elif type_is is 'azure':
            self.k4a = PyK4A()
            self.k4a.start()
        rospy.Subscriber('/camera/depth/image_raw', Image, self.callback1)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.callback2)

    def is_ready(self):
        if self.k_depth_ready and self.k_rgb_ready:
            return True
        else:
            return False

    def callback1(self, msg):
        """Depth data from kinect"""
        if self.type == 'xbox':
            img = self.bridge.imgmsg_to_cv2(msg)
            cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
            self.k_depth_ready = True
        elif self.type == 'azure':
            img = self.k4a.get_capture()
            img = img.depth
            cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
            self.k_depth_ready = True
        if self.is_streaming:
            self.depth_buffer.append(np.round(img).astype(np.uint8))
        if self.depth_buffer.__len__() > BUFFER_THRESHOLD:
            print("Depth Buffer full")
            pass

    def callback2(self, msg):
        """RGB data from kinect"""
        if self.type == 'xbox':
            img = self.bridge.imgmsg_to_cv2(msg)
            self.k_rgb_ready = True
        elif self.type == 'azure':
            img = self.k4a.get_capture()
            img = img.color
            self.k_rgb_ready = True
        if self.is_streaming:
            self.rgb_buffer.append(np.round(img).astype(np.uint8))
        if self.rgb_buffer.__len__() > BUFFER_THRESHOLD:
            print("RGB Buffer full")
            pass

    def img_show(self):
        if len(self.rgb_buffer) > 0:
            cv2.imshow('xbox_kinext_depth', self.rgb_buffer[0])
            cv2.waitKey(1)
        else:
            pass
