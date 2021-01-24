from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv2
import numpy as np


def doloop():
    global depth, rgb
    while True:
        # Get a fresh frame
        (depth, _), (rgb, _) = get_depth(), get_video()

        # Build a two panel color image
        cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        depth = np.round(depth).astype(np.uint8)

        # Simple Downsample
        cv2.imshow("test", depth)
        cv2.waitKey(5)


doloop()
