import numpy as np
from app.utils import BUFFER_EMPTY_THRESHOLD


class Stream:

    def __init__(self, sensor=None, kinect=None):
        self.sensor = sensor
        self.kinect = kinect
        self.sensor_ignore = False
        self.buffer_ignore = False
        self.sensor_buffer_empty_count = 0
        self.video_buffer_empty_count = 0

    def get_data(self):
        data = list()
        for index, sensor in enumerate(self.sensor):
            if sensor.msg_buffer.__len__() is not 0:
                try:
                    msg = sensor.msg_buffer[0].replace("[", "")
                    msg = msg.replace("]", "")
                    msg = msg.split(",")
                    msg = [float(i) for i in msg]
                    data.append(msg)
                    sensor.msg_buffer.pop(0)
                    # print("poping element --------------------------------")
                except BufferError:
                    if self.sensor_ignore:
                        print("passing sensor data overflow error on sensor",
                              index)
                    else:
                        print("Sensor data overflow error on sensor", index)
                        raise BufferError
            else:
                print(f"Sensor-{index+1} data buffer is currently empty")
                self.sensor_buffer_empty_count += 1
                if self.sensor_buffer_empty_count > BUFFER_EMPTY_THRESHOLD:
                    raise BufferError
        return data

    def get_video_stream(self):
        video = list()
        depth = list()
        for index, kinect in enumerate(self.kinect):
            if kinect.rgb_buffer.__len__() is not 0 and \
                    kinect.depth_buffer.__len__() is not 0:
                try:
                    dp = kinect.depth_buffer[0]
                    vd = kinect.rgb_buffer[0]
                    kinect.depth_buffer.pop(0)
                    kinect.rgb_buffer.pop(0)
                    video.append(vd)
                    depth.append(dp)
                    # print("poping video --------------------------------")
                except BufferError:
                    if self.buffer_ignore:
                        print("passing kinect data overflow error on sensor",
                              index)
                    else:
                        print("Kinect data overflow error on sensor", index)
                        raise BufferError
            else:
                print(f"Kinect-{index+1} data buffer is currently empty")
                self.video_buffer_empty_count += 1
                return False, False
                if self.video_buffer_empty_count > BUFFER_EMPTY_THRESHOLD:
                    raise BufferError
        return video, depth

    def set_error(self, s_ignore, b_ignore):
        self.sensor_ignore = s_ignore
        self.buffer_ignore = b_ignore
