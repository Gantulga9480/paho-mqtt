# One and only broker
BROKER = "192.168.1.2"

# Client list

# Topic list
PC_SENSOR_CONTROL = "pc/sensor-start"
CLEINT_1 = "sensors/sensor1/status"
CLEINT_2 = "sensors/sensor2/status"
CLEINT_3 = "sensors/sensor3/status"
CLEINT_4 = "sensors/sensor4/status"
CLEINT_5 = "sensors/sensor5/status"
CLEINT_6 = "sensors/sensor6/status"

SENSOR_1 = "sensors/sensor1/data"
SENSOR_2 = "sensors/sensor2/data"
SENSOR_3 = "sensors/sensor3/data"    
SENSOR_4 = "sensors/sensor4/data"
SENSOR_5 = "sensors/sensor5/data"
SENSOR_6 = "sensors/sensor6/data"

# Command list
START_COMMAND = "2" # Sensor start
WAIT_COMMAND = "1" # Sensor stop
STOP_COMMAND = "0" # Sensor exit
SENSOR_MONITOR = "sensor_monitor.py"

# Path list

# Add or remove activity here
ACTIVITIES = ["activity_1", "activity_2"]

# Const
SPEED = 100
FILEFORMAT = "%Y_%m_%d_%H_%M_%S"
FOLDERFORMAT = ""

# Message
FILE_FOUND_MSG = "Same file found\nOverwrite existing file?"
