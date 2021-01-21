import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image


class ImgDisp:

    def __init__(self):
        self.bridge = CvBridge()
        # self.pub = rospy.Publisher('/cv2_depth_img_raw', Image, queue_size=1)

    def callback1(self, msg):
        print(len(msg.data))
        # print(ord(msg.data))

    def callback(self, msg):
        print(msg.height)
        print(msg.width)
        img = self.bridge.imgmsg_to_cv2(msg)
        image = np.array(img, dtype=np.float32)
        cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
        image = np.round(image).astype(np.uint8)
        # cv2.circle(image, (200, 300), 5, (255, 255, 255), 3)
        cv2.imshow('azure_kinect', image)
        cv2.waitKey(1)

    def main(self):
        rospy.init_node('img_disp_node')
        rospy.Subscriber('depth/image_raw', Image, self.callback)
        rospy.spin()


if __name__ == '__main__':
    img_disp = ImgDisp()
    img_disp.main()
