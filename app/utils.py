# One and only broker
# BROKER = "192.168.1.60"
BROKER = "127.0.0.1"

# Topic list
SENSORS = ["sensors/sensor1/data",
           "sensors/sensor2/data",
           "sensors/sensor3/data",
           "sensors/sensor4/data",
           "sensors/sensor5/data",
           "sensors/sensor6/data"]

# Command list

# Path list

# Add or remove activity here
ACTIVITIES = ["activity_1", "activity_2"]

# Const
DATA_SPEED = 100 # ms
VIDEO_SPEED = 33 # ms
FILEFORMAT = "%Y_%m_%d_%H_%M_%S"

# Message
FILE_FOUND_MSG = "Same file found\nOverwrite existing file?"
SENSOR_DATA_ERROR = "Not recieving data from sensor"
