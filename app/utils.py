# One and only broker
# BROKER = "192.168.0.100"
BROKER = "127.0.0.1"

# Topic list
SENSORS = [["sensors/sensor1/data", 1, 1],
           ["sensors/sensor2/data", 2, 1],
           ["sensors/sensor3/data", 3, 1],
           ["sensors/sensor4/data", 4, 1],
           ["sensors/sensor5/data", 5, 1],
           ["sensors/sensor6/data", 6, 1],
           ["sensors/sensor7/data", 7, 1],
           ["sensors/sensor8/data", 10, 0],
           ["sensors/sensor9/data", 9, 1],
           ["sensors/sensor10/data", 8, 1]]

# Command list
START = 'start'
STOP = 'stop'
RESET = 'reset'
SAVE = 'save'
PLAY = 'play'
QUIT = 'quit'
ACTIVITIE_START = 'a_start'
ACTIVITIE_STOP = 'a_stop'

# Path list
SAVE_PATH = "data"
CACHE_PATH = "cache"


# Add or remove activity here 
# A list
#ACTIVITIES = ["Dressing",
#              "Watching TV",
#              'Reading',
#              'Working computer',
#              'Standing',
#              'Exercising',
#              'Walking in room',
#              'Meal preparation',
#              'Eating',
#              'Wash dishes',
#              'Clean room',
#              'Sleeping',
#              'Falling',
#              'Respiration',
#              'Enter room',
#              'Talk to phone']

# B list
#ACTIVITIES = ['Enter room',
#              "Dressing",
#               "Having rest",
#               'Working computer',
#               'Reading',
#               "Watching TV",
#               'Standing',
#              'Exercising',
#              'Respiration',
#              'Walking in room',
#               'Meal preparation',
#               'Eating',
#               'Wash dishes',
#               'Clean room',
#               'Sleeping',
#               'Falling',
#               'Talk to phone']

# C list           
#ACTIVITIES = ["Dressing",
#              "Watching TV",
#              "Having rest",
#               'Meal preparation',
#               'Eating',
#               'Wash dishes',
#               'Exercising',
#               'Enter room',
#               'Walking in room',
#               'Standing',
#               'Reading',
#               'Clean room',
#               'Working computer',
#               'Sleeping',
#               'Respiration',
#               'Talk to phone',
#               'Falling']

# D list
ACTIVITIES = ["Dressing",
               'Clean room',
               "Watching TV",
               'Meal preparation',
               'Eating',
               'Wash dishes',
              'Walking in room',
              'Exercising',
               'Standing',
               "Having rest",
               'Working computer',
               'Reading',
              'Enter room',
               'Sleeping',
              'Respiration',
               'Talk to phone',
               'Falling']


# Const
SUB_DURATION = 2  # sec
VIDEO_SPEED = 15  # ms
CAMERA_SPEED = 33
AZURE_KINECT_DEPTH_SIZE = (640, 576)
AZURE_KINECT_RGB_SIZE = (1280, 720)
FPS = 30
TIME_FORMAT = "%H_%M_%S"
DATE_FORMAT = "%Y_%m_%d"
DATE_TIME = "%Y_%m_%d_%H_%M_%S"

# Message
SENSOR_ERROR = "Sensor is not ready, check log"


class Color:

    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)


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
    delta = str(stop - start)
    time = delta.split(":")
    if raw:
        h = int(time[0])
        m = int(time[1])
        s = int(float(time[2].lstrip("0")))
        ms = int((float(time[2].lstrip("0")) - s)*1000)
        return h, m, s, ms
    else:
        h = time[0]
        m = time[1]
        s = int(float(time[2].lstrip("0")))
        ms = int((float(time[2].lstrip("0")) - s)*1000)
        return f"{h.zfill(2)}:{m.zfill(2)}:{str(s).zfill(2)},{str(ms)}"


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


def read(path, mod):
    f = open(path, mod)
    a = f.read()
    f.close()
    return a


def write(a):
    f = open('app/usr/user_index.txt', 'w')
    f.write(str(a))


class SensorDeathError(Exception):
    """Raised when client death_counter > 4"""
    def __init__(self, msg):
        print(f'SENSOR {msg} disconnected')
