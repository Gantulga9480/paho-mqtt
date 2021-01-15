import paho.mqtt.client as mqtt
import time


def on_connect(client, userdata, level, buf):
    pass


def on_message(client, userdata, message):
    pass


def on_publish(client, userdata, result):
    print("sent 1")


def on_disconnect(client, userdata, rc):
    print("client disconnected ok")


def wait_for_publish():
    pass


broker = "192.168.1.2"
client = mqtt.Client(f"sensor_control_node")
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.on_disconnect = on_disconnect
client.wait_for_publish = wait_for_publish
client.connect(broker)
client.loop_start()
time.sleep(0.1)
pub = client.publish("pc/sensor-start", payload="1", qos=2)
time.sleep(0.1)
client.disconnect()
