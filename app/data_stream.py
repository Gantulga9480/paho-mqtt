import paho.mqtt.client as mqtt
import numpy as np


class Stream:

    def __init__(self, clients=None):
        self.clients = clients

    def get_data(self):
        data = list()
        print("getting data")
        for client in self.clients:
            msg = client.msg.replace("[", "")
            msg = msg.replace("]", "")
            msg = msg.split(",")
            msg = [float(i) for i in msg]
            data.append(msg)
        return data
