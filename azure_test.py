from pyk4a import PyK4A
import cv2
import numpy as np

# Load camera with the default config
k4a = PyK4A()
k4a.start()

# Get the next capture (blocking function)
while True:
    capture = k4a.get_capture()
    # img_color = capture.depth
    img_color = capture.color
    # Display with pyplot
    # cv2.normalize(img_color, img_color, 0, 255, cv2.NORM_MINMAX)
    img_color = np.round(img_color).astype(np.uint8)
    cv2.imshow("img", img_color)
    cv2.waitKey(1)
