import numpy as np
from app.utils import BUFFER_EMPTY_THRESHOLD
from app.utils import BUFFER_THRESHOLD


class Stream:

    def __init__(self, sensor=None):
        self.sensor = sensor
        self.sensor_ignore = False
        self.buffer_ignore = False
        self.sensor_buffer_empty_count = 0
        self.video_buffer_empty_count = 0

    def get_data(self):
        data = list()
        for index, sensor in enumerate(self.sensor):
            msg_len = len(sensor.msg_buffer)
            if msg_len is not 0:
                msg = sensor.msg_buffer[0]
                if msg_len >= 2:
                    sensor.msg_buffer.pop(0)
                msg = msg.replace("[", "")
                msg = msg.replace("]", "")
                msg = msg.split(",")
                msg = [float(i) for i in msg]
                data.append(msg)
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

    def set_error(self, s_ignore, b_ignore):
        self.sensor_ignore = s_ignore
        self.buffer_ignore = b_ignore
