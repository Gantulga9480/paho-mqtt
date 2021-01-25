# One and only broker
# BROKER = "192.168.0.100"
BROKER = "127.0.0.1"

# Topic list
SENSORS = ["sensors/sensor1/data"]

# KINECTS = [['kinect_1', 'xbox']]

# Command list

# Path list
SAVE_PATH = r"path"

# Add or remove activity here
ACTIVITIES = ["activity_1", "activity_2"]

# Const
SUB_DURATION = 1
DATA_SPEED = 100  # ms
VIDEO_SPEED = 34  # ms
BUFFER_THRESHOLD = 30
BUFFER_EMPTY_THRESHOLD = 30
XBOX_KINECT_FRAME_SIZE = (640, 480)
AZURE_KINECT_DEPTH_SIZE = (640, 576)
AZURE_KINECT_RGB_SIZE = (1280, 720)
TIME_FORMAT = "%H:%M:%S"
DATE_FORMAT = "%Y_%m_%d"
DATE_TIME = "%Y_%m_%d_%H_%M_%S"

SRT_FORMAT = "%H:%M:%S,000"

# Message
FILE_FOUND_MSG = "Same file found\nOverwrite existing file?"
BUFFER_ERROR = "Buffer error occured chech log"
SENSOR_ERROR = "Sensor is not ready, check"
KINECT_ERROR = "Kinect is not ready, check"


class Color:

    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)


class CsvRead:

    def __init__(self):
        import csv
        import numpy as np

    def read(self):
        file_1 = open("")
        data = []
        with file_1:
            csv_reader = csv.reader(file_1, delimiter=',')
            for i, row in enumerate(csv_reader):
                if i == 0:
                    pass
                else:
                    data = row[1].replace("[", "").replace("]", "")
                    data = data.replace(" ", "").split(",")

            data_1 = np.zeros((len(data), 64, 64))
            for i in range(len(data)):
                for j in range(8):
                    for k in range(8):
                        data_1[i][j][k] = data[i*j]


def get_time(sec, raw=False):
    sec = round(sec, 3)
    mins = sec // 60
    sec_s = int(sec % 60)
    ms = int((sec - sec_s)*1000)
    hours = int(mins // 60)
    mins = int(mins % 60)
    if raw:
        return hours, mins, sec_s, ms
    else:
        h = str(hours).zfill(2)
        m = str(mins).zfill(2)
        s = str(sec_s).zfill(2)
        ms = str(ms).zfill(3)
        return f"{h}:{m}:{s},{ms}"


def get_time_date(start=None, stop=None, raw=False):
    delta = stop - start
    delta = str(delta)
    time = delta.split(":")
    h = int(time[0])
    m = int(time[1])
    s = int(float(time[2].lstrip("0")))
    ms = int((float(time[2].lstrip("0")) - s)*1000)
    if raw:
        return h, m, s, ms
    else:
        f"{str(h).zfill(2)}:{str(m).zfill(2)}:{str(s).zfill(2)},{str(ms)}"


def get_time_1(time, raw=True):
    s = time // 1000
    m = s // 60
    h = m // 60
    s = s % 60
    m = m % 60
    ms = time % 1000
    if raw:
        return h, m, s, ms
    else:
        return f"{str(h).zfill}"


def get_time_2(date):
    time = date.strftime(TIME_FORMAT)
    time = time.split(":")
    h = int(time[0])
    m = int(time[1])
    s = int(float(time[2].lstrip("0")))
    ms = int((float(time[2].lstrip("0")) - s)*1000)
    return h, m, s, ms
