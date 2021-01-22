import numpy as np
from app.utils import BUFFER_EMPTY_THRESHOLD
from app.utils import BUFFER_THRESHOLD


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
            msg_len = sensor.msg_buffer.__len__()
            if msg_len is not 0:
                msg = sensor.msg_buffer[0]
                sensor.msg_buffer.pop(0)
                data.append(msg)
                # print("poping element --------------------------------")
                if msg_len > BUFFER_THRESHOLD:
                    if self.sensor_ignore:
                        pass
                    else:
                        print(f"Overflowe at {sensor.info}: len {msg_len}")
                        raise BufferError
            else:
                print(f"Sensor-{index+1} data buffer is currently empty")
                self.sensor_buffer_empty_count += 1
                if self.sensor_buffer_empty_count > BUFFER_EMPTY_THRESHOLD:
                    print(f"Not connected to {sensor.info}")
                    raise BufferError
        return data

    def get_video_stream(self):
        video = list()
        depth = list()
        for index, kinect in enumerate(self.kinect):
            rgb_len = kinect.rgb_buffer.__len__()
            dp_len = kinect.depth_buffer.__len__()
            if rgb_len is not 0 and dp_len is not 0:
                dp = kinect.depth_buffer[0]
                vd = kinect.rgb_buffer[0]
                kinect.depth_buffer.pop(0)
                kinect.rgb_buffer.pop(0)
                video.append(vd)
                depth.append(dp)
                # print("poping video --------------------------------")
                if rgb_len > BUFFER_THRESHOLD or dp_len > BUFFER_THRESHOLD:
                    if self.buffer_ignore:
                        pass
                    else:
                        print("Kinect data overflow error on", index)
                        print(f"rgb: {rgb_len}, depth: {dp_len}")
                        raise BufferError
            else:
                print(f"Kinect-{index+1} data buffer is currently empty")
                self.video_buffer_empty_count += 1
                return False, False
                if self.video_buffer_empty_count > BUFFER_EMPTY_THRESHOLD:
                    print(f"Not connected to {kinect.id_name}:{kinect.type}")
                    raise BufferError
        return video, depth

    def set_error(self, s_ignore, b_ignore):
        self.sensor_ignore = s_ignore
        self.buffer_ignore = b_ignore
