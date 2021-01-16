import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Image
from sensor_control import PahoMqtt as mqtt
from sensor_control import BROKER


class ImgDisp:

    def __init__(self):
        self.bridge = CvBridge()

    def callback1(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        image = np.array(img, dtype=np.float32)
        cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
        image = np.round(image).astype(np.uint8)
        # cv2.circle(image, (200, 300), 5, (255, 255, 255), 3)
        cv2.imshow('xbox_kinext_depth', image)
        cv2.waitKey(1)

    def callback2(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        image = np.array(img, dtype=np.float32)
        cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
        image = np.round(image).astype(np.uint8)
        # cv2.circle(image, (200, 300), 5, (255, 255, 255), 3)
        cv2.imshow('xbox_kinext_color', image)
        cv2.waitKey(1)

    def main(self):
        rospy.init_node('img_disp_node')
        rospy.Subscriber('camera/depth/image_raw', Image, self.callback1)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.callback2)
        rospy.spin()


if __name__ == '__main__':
    img_disp = ImgDisp()
    img_disp.main()