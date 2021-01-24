from pyk4a import PyK4A
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    k4a = PyK4A()
    k4a.start()
    print("azure kinect ready")
    while True:
        capture = k4a.get_capture()
        img_color = capture.color
        img_depth = capture.depth
        cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
        img_color = np.round(img_color).astype(np.uint8)
        img_depth = np.round(img_depth).astype(np.uint8)
        cv2.imshow("azure_1", img_depth)
        cv2.waitKey(5)


if __name__ == '__main__':
    main()
