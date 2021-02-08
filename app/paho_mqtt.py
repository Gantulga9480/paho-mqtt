import paho.mqtt.client as mqtt
from datetime import datetime as dt
from app.mqtt import Mqtt
import csv


class PahoMqtt(Mqtt):

    def __init__(self, broker, info, port=1883,
                 raw_msg=False, c_msg="", d_msg=""):
        super().__init__(broker, info, port=port, raw_msg=raw_msg,
                         c_msg=c_msg, d_msg=d_msg)
        self.is_streaming = False
        self.is_started = False
        self.sensor_ready = False
        self.label = None
        self.counter = 0
        self.counter_temp = 0
        self.death_counter = 0

    def stream_init(self, path):
        self._file = open(f'{path}/sensor_{self.info}.csv', "w+", newline='')
        self._writer = csv.writer(self._file)
        self.is_streaming = True
        self.is_started = True

    def stream_stop(self):
        self.is_streaming = False
        self.is_started = False

    def _on_message(self, client, userdata, message):
        self.counter += 1
        if self.counter > 10000:
            self.counter = 0
        self.sensor_ready = True
        if self.is_streaming:
            msg = message.payload.decode("utf-8", "ignore")
            msg = msg.replace("[", "")
            msg = msg.replace("]", "")
            msg = msg.replace(" ", "")
            if self.label:
                self._writer.writerow([dt.now(), msg, self.label])
                self.label = None
            else:
                self._writer.writerow([dt.now(), msg, 0])
