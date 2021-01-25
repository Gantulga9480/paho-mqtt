from pyk4a import PyK4A
# from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv2
import numpy as np
# import rospy
import time
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from app.utils import AZURE_KINECT_DEPTH_SIZE, AZURE_KINECT_RGB_SIZE


def main():

    azure_rgb_out = cv2.VideoWriter("./ak2rgb.avi",
                                    cv2.VideoWriter_fourcc(*'MP42'),
                                    30, (640, 576))
    """
    azure_depth_out = cv2.VideoWriter("a_k2_depth.avi",
                                      cv2.VideoWriter_fourcc(*'MP42'),
                                      30, (640, 576))
    """
    #   (depth, _), (rgb, _) = get_depth(), get_video()
    k4a = PyK4A()
    k4a.start()
    time.sleep(1)
    frame_depth = list()
    frame_rgb = list()
    print("azure kinect ready")
    for i in range(600):
        # (depth, _), (rgb, _) = get_depth(), get_video()
        capture = k4a.get_capture()
        img_color = capture.depth
        # img_color = img_color[:, :, 2::-1]
        # img_color = img_color[:, :, 2::-1]
        # print(img_color.shape)
        # img_depth = capture.depth
        # cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        # depth = np.round(depth).astype(np.uint8)
        cv2.normalize(img_color, img_color, 0, 255, cv2.NORM_MINMAX)
        img_color = img_color.astype(np.uint8)
        # img_color = np.round(img_color)
        img_color = cv2.cvtColor(img_color, cv2.COLOR_GRAY2RGB)
        # img_depth = np.round(img_depth).astype(np.uint8)
        azure_rgb_out.write(img_color)
        # azure_depth_out.write(img_depth)
        # cv2.imshow("kinect_1", depth)
    azure_rgb_out.release()
    k4a.stop()
    time.sleep(1)
    # azure_depth_out.release()


if __name__ == '__main__':
    main()
