from pyk4a import PyK4A
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    bridge = CvBridge()
    k4a = PyK4A()
    k4a.start()
    rospy.init_node('azure_kinect_node', anonymous=True)
    pub = rospy.Publisher('azure_color', Image, queue_size=1)
    pub_x = rospy.Publisher('azure_depth', Image, queue_size=1)
    rate = rospy.Rate(30)
    print("azure kinect ready")
    while not rospy.is_shutdown():
        capture = k4a.get_capture()
        img_color = capture.color
        img_depth = capture.depth
        cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
        img_color = np.round(img_color).astype(np.uint8)
        img_depth = np.round(img_depth).astype(np.uint8)
        img = bridge.cv2_to_imgmsg(img_color)
        img_x = bridge.cv2_to_imgmsg(img_depth)
        pub.publish(img)
        pub_x.publish(img_x)
        rate.sleep()


if __name__ == '__main__':
    main()
