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
        self.counts = [0 for _ in self.sensor]

    def get_data(self):
        data = list()
        is_full = True
        for index, sensor in enumerate(self.sensor):
            msg_len = len(sensor.msg_buffer)
            if 0 < msg_len < BUFFER_THRESHOLD:
                msg = sensor.msg_buffer.pop(0)
                msg = msg.replace("[", "")
                msg = msg.replace("]", "")
                msg = msg.replace(" ", "")
                sensor.temp = msg
                data.append(msg)
                self.counts[index] = 0
            elif msg_len == 0 and sensor.temp:
                data.append(sensor.temp)
                sensor.temp = None
            elif msg_len > BUFFER_THRESHOLD:
                if self.sensor_ignore:
                    pass
                else:
                    print(f"Overflowe at {sensor.info} : len {msg_len}")
                    raise BufferError
            else:
                is_full = False
                # data.append('Sensor disconnected')
                print(f"Sensor-{index+1} data buffer is currently empty")
                self.counts[index] += 1
                print(self.counts[index])
                if self.counts[index] > BUFFER_EMPTY_THRESHOLD:
                    print(f"Not connected to {sensor.info}")
                    raise BufferError
        return data, is_full

    def set_error(self, s_ignore, b_ignore):
        self.sensor_ignore = s_ignore
        self.buffer_ignore = b_ignore
