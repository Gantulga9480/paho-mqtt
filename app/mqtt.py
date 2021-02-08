import paho.mqtt.client as mqtt


class Mqtt:

    def __init__(self, broker, info, port=1883,
                 raw_msg=False, c_msg="", d_msg=""):
        self.__broker = broker
        self.__port = port
        self.info = info
        self._c_msg = c_msg
        self._d_msg = d_msg
        self.__client = mqtt.Client(f"{info} control")
        if not raw_msg:
            self.__client.on_message = self._on_message
        else:
            self.__client.on_message = self._on_message_raw
        self.__client.on_connect = self._on_connect
        self.__client.on_publish = self._on_publish
        self.__client.on_disconnect = self._on_disconnect
        self.__client.wait_for_publish = self._wait_for_publish
        self.__client.connect(self.__broker, self.__port)

    def _on_connect(self, client, userdata, level, buf):
        print(f"{self._c_msg} connected")

    def _on_message(self, client, userdata, message):
        pass

    def _on_message_raw(self, client, userdata, message):
        pass

    def _on_publish(self, client, userdata, result):
        print('command published')

    def _on_disconnect(self, client, userdata, rc):
        self.sensor_ready = False
        print(f"{self._d_msg} disconnected")

    def _wait_for_publish(self):
        print('waiting to publish command')

    def disconnect(self):
        self.__client.disconnect()

    def publish(self, topic, msg, qos=0):
        self.__client.publish(topic, payload=msg, qos=qos)

    def subscribe(self, topic, qos=0):
        self.__client.subscribe(topic, qos=qos)

    def loop_start(self):
        self.__client.loop_start()
