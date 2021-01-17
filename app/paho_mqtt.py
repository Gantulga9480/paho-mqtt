import paho.mqtt.client as mqtt


class PahoMqtt:

    def __init__(self, broker, info, port=1883):
        self.__broker = broker
        self.__port = port
        self.msg = None
        self.__client = mqtt.Client(f"sensor_control_{info}")
        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.on_publish = self.__on_publish
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.wait_for_publish = self.__wait_for_publish
        self.__client.connect(self.__broker, self.__port)

    def __on_connect(self, client, userdata, level, buf):
        print("client connected")

    def __on_message(self, client, userdata, message):
        self.msg = message.payload.decode("utf-8", "ignore")

    def __on_publish(self, client, userdata, result):
        print("command sent")

    def __on_disconnect(self, client, userdata, rc):
        print("client disconnected")

    def __wait_for_publish(self):
        pass

    def disconnect(self):
        self.__client.disconnect()

    def publish(self, topic, msg, qos=2):
        self.__client.publish(topic, payload=msg, qos=qos)

    def subscribe(self, topic, qos=2):
        self.__client.subscribe(topic, qos=qos)

    def loop_start(self):
        self.__client.loop_start()