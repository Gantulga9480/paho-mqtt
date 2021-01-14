import paho.mqtt.client as mqtt
import time


def on_connect(client, userdata, level, buf):
    pass


def on_message(client, userdata, message):
    print("msg", message.payload.decode("utf-8", "ignore"))


broker = "192.168.1.2"
client = mqtt.Client("sensor1_read")
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker)
time.sleep(1)

client.subscribe("sensors/sensor1_data", 2)
client.loop_forever()
time.sleep(1)
