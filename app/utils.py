# One and only broker
# BROKER = "192.168.0.100"
BROKER = "127.0.0.1"

# Topic list
SENSORS = ["sensors/sensor1/data",
           "sensors/sensor2/data",
           "sensors/sensor3/data",
           "sensors/sensor4/data",
           "sensors/sensor5/data",
           "sensors/sensor6/data"]

KINECTS = [['kinect_1', 'xbox'], ['azure_1', 'azure']]

# Command list

# Path list
SAVE_PATH = r"path"

# Add or remove activity here
ACTIVITIES = ["activity_1", "activity_2"]

# Const
SUB_DURATION = 2
DATA_SPEED = 100  # ms
VIDEO_SPEED = 33  # ms
BUFFER_THRESHOLD = 5
BUFFER_EMPTY_THRESHOLD = 5
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


def get_time(sec, raw=False):
    mins = sec // 60
    sec = int(round(sec % 60))
    hours = int(mins // 60)
    mins = int(mins % 60)
    if raw:
        return hours, mins, sec
    else:
        h = str(hours).zfill(2)
        m = str(mins).zfill(2)
        s = str(sec).zfill(2)
        return f"{h}:{m}:{s},000"
