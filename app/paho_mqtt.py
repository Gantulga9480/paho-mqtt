import paho.mqtt.client as mqtt


class PahoMqtt:

    def __init__(self, broker, info, port=1883,
                 raw_msg=False, c_msg="", d_msg=""):
        self.__broker = broker
        self.__port = port
        self.info = info
        self.msg_buffer = list()
        self.is_streaming = False
        self.sensor_ready = False
        self.c_msg = c_msg
        self.d_msg = d_msg
        self.__client = mqtt.Client(f"sensor_control_{info}")
        if not raw_msg:
            self.__client.on_message = self.__on_message
        else:
            self.__client.on_message = self.__on_message_raw
        self.__client.on_connect = self.__on_connect
        self.__client.on_publish = self.__on_publish
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.wait_for_publish = self.__wait_for_publish
        self.__client.connect(self.__broker, self.__port)

    def __on_connect(self, client, userdata, level, buf):
        print(f"{self.c_msg} connected")

    def __on_message(self, client, userdata, message):
        self.sensor_ready = True
        msg = message.payload.decode("utf-8", "ignore")
        if self.is_streaming:
            self.msg_buffer.append(msg)

    def __on_message_raw(self, client, userdata, message):
        self.sensor_ready = True
        if self.is_streaming:
            self.msg_buffer.append(message.payload)

    def __on_publish(self, client, userdata, result):
        pass

    def __on_disconnect(self, client, userdata, rc):
        self.sensor_ready = False
        print(f"{self.d_msg} disconnected")

    def __wait_for_publish(self):
        pass

    def disconnect(self):
        self.__client.disconnect()

    def publish(self, topic, msg, qos=0):
        self.__client.publish(topic, payload=msg, qos=qos)

    def subscribe(self, topic, qos=0):
        self.__client.subscribe(topic, qos=qos)

    def loop_start(self):
        self.__client.loop_start()
