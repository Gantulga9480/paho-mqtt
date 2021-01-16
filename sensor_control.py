import paho.mqtt.client as mqtt
import time
import os
import numpy as np
from tkinter import *
from tkinter import ttk


BROKER = "192.168.1.2"
PC_SENSOR_CONTROL = "pc/sensor-start"
START = "2"
WAIT = "1"
STOP = "0"


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


class SensorControl(Tk):

    def __init__(self, screenName=None, baseName=None,
                 useTk=1, sync=0, use=None):
        super().__init__(screenName=screenName, baseName=baseName,
                         useTk=useTk, sync=sync, use=use)
        self.mqtt = PahoMqtt(BROKER, "master")
        self.mqtt.loop_start()

        self.mqtt1 = PahoMqtt(BROKER, "1")
        self.mqtt1.subscribe("sensors/sensor1_status")
        self.mqtt1.loop_start()

        self.mqtt2 = PahoMqtt(BROKER, "2")
        self.mqtt2.subscribe("sensors/sensor2_status")
        self.mqtt2.loop_start()

        self.mqtt3 = PahoMqtt(BROKER, "3")
        self.mqtt3.subscribe("sensors/sensor3_status")
        self.mqtt3.loop_start()

        self.mqtt4 = PahoMqtt(BROKER, "4")
        self.mqtt4.subscribe("sensors/sensor4_status")
        self.mqtt4.loop_start()

        self.mqtt5 = PahoMqtt(BROKER, "5")
        self.mqtt5.subscribe("sensors/sensor5_status")
        self.mqtt5.loop_start()

        self.mqtt6 = PahoMqtt(BROKER, "6")
        self.mqtt6.subscribe("sensors/sensor6_status")
        self.mqtt6.loop_start()

        self.title("Control")
        self.resizable(True, True)
        self.configure(bg='white')

        s = ttk.Style()
        s.configure("Red.TLabel", foreground='red')
        s.configure("Green.TLabel", foreground='green')
        s.configure("Black.TLabel", foreground='black')
        s.configure("Yellow.TLabel", foreground='yellow')
        s.configure('White.TLabelFrame', background='white')

        # Frame 1
        self.sensor_frame1 = LabelFrame(self, text="Sensor control",
                                        background='white')
        self.sensor_frame1.pack()

        self.label_sensor_1 = Label(self.sensor_frame1, text="SENSOR 1",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_1.grid(row=0, column=0, columnspan=2)

        self.label_sensor_2 = Label(self.sensor_frame1, text="SENSOR 2",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_2.grid(row=1, column=0, columnspan=2)

        self.label_sensor_3 = Label(self.sensor_frame1, text="SENSOR 3",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_3.grid(row=2, column=0, columnspan=2)

        self.label_sensor_4 = Label(self.sensor_frame1, text="SENSOR 4",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_4.grid(row=3, column=0, columnspan=2)

        self.label_sensor_5 = Label(self.sensor_frame1, text="SENSOR 5",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_5.grid(row=4, column=0, columnspan=2)

        self.label_sensor_6 = Label(self.sensor_frame1, text="SENSOR 6",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_6.grid(row=5, column=0, columnspan=2)

        self.label_sensor_7 = Label(self.sensor_frame1, text="KINECT 1",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_7.grid(row=6, column=0, columnspan=2)

        self.label_sensor_8 = Label(self.sensor_frame1, text="KINECT 2",
                                    background='white',
                                    font=("default", 15, 'bold'))
        self.label_sensor_8.grid(row=7, column=0, columnspan=2)
        self.start_btn = ttk.Button(self.sensor_frame1,
                                    text="Start", command=self.start)
        self.start_btn.grid(row=8, column=0)
        self.stop_btn = ttk.Button(self.sensor_frame1,
                                   text="Stop", command=self.stop)
        self.stop_btn.grid(row=8, column=1)

        # Frame 2
        self.sensor_frame2 = LabelFrame(self, text="Data control",
                                        background='white')
        self.sensor_frame2.pack()
        self.data_stream_start_btn = ttk.Button(self.sensor_frame2,
                                                text="Steam start",
                                                command=self.stream_start)
        self.data_stream_start_btn.grid(row=0, column=0)
        self.data_stream_stop_btn = ttk.Button(self.sensor_frame2,
                                               text="Steam stop",
                                               command=self.stream_stop)
        self.data_stream_stop_btn.grid(row=0, column=1)

        # Menu
        menubar = Menu(self)
        """
        filemenu = Menu(menubar, tearoff=0)
        filemenu.add_command(label="Sensor 1",
                             command=lambda: self.show_data(1))
        filemenu.add_command(label="Sensor 2",
                             command=lambda: self.show_data(2))
        filemenu.add_command(label="Sensor 3",
                             command=lambda: self.show_data(3))
        filemenu.add_command(label="Sensor 4",
                             command=lambda: self.show_data(4))
        filemenu.add_command(label="Sensor 5",
                             command=lambda: self.show_data(5))
        filemenu.add_command(label="Sensor 6",
                             command=lambda: self.show_data(6))
        filemenu.add_command(label="Kinect 1")
        filemenu.add_command(label="Kinect 2")
        menubar.add_cascade(label="Visual", menu=filemenu)
        """
        tool = Menu(menubar, tearoff=0)
        tool.add_command(label="Disconnect sensors", command=self.close)
        menubar.add_cascade(label="Tools", menu=tool)
        self.config(menu=menubar)

        # check sensors
        self.check_sensor_1()
        self.check_sensor_2()
        self.check_sensor_3()
        self.check_sensor_4()
        self.check_sensor_5()
        self.check_sensor_6()
        
        self.mainloop()

    def check_sensor_1(self):
        if self.mqtt1.msg is not None:
            if self.mqtt1.msg == "c":
                self.label_sensor_1["foreground"] = 'green'
            elif self.mqtt1.msg == "w":
                self.label_sensor_1["foreground"] = 'blue'
            elif self.mqtt1.msg == "d":
                self.label_sensor_1["foreground"] = 'red'
        self.mqtt1.msg = None
        self.after(10, self.check_sensor_1)

    def check_sensor_2(self):
        if self.mqtt2.msg is not None:
            if self.mqtt2.msg == "c":
                self.label_sensor_2["foreground"] = 'green'
            elif self.mqtt2.msg == "w":
                self.label_sensor_2["foreground"] = 'blue'
            elif self.mqtt2.msg == "d":
                self.label_sensor_2["foreground"] = 'red'
        self.mqtt2.msg = None
        self.after(10, self.check_sensor_2)

    def check_sensor_3(self):
        if self.mqtt3.msg is not None:
            if self.mqtt3.msg == "c":
                self.label_sensor_3["foreground"] = 'green'
            elif self.mqtt3.msg == "w":
                self.label_sensor_3["foreground"] = 'blue'
            elif self.mqtt3.msg == "d":
                self.label_sensor_3["foreground"] = 'red'
        self.mqtt3.msg = None
        self.after(10, self.check_sensor_3)

    def check_sensor_4(self):
        if self.mqtt4.msg is not None:
            if self.mqtt4.msg == "c":
                self.label_sensor_4["foreground"] = 'green'
            elif self.mqtt4.msg == "w":
                self.label_sensor_4["foreground"] = 'blue'
            elif self.mqtt4.msg == "d":
                self.label_sensor_4["foreground"] = 'red'
        self.mqtt4.msg = None
        self.after(10, self.check_sensor_4)

    def check_sensor_5(self):
        if self.mqtt5.msg is not None:
            if self.mqtt5.msg == "c":
                self.label_sensor_5["foreground"] = 'green'
            elif self.mqtt5.msg == "w":
                self.label_sensor_5["foreground"] = 'blue'
            elif self.mqtt5.msg == "d":
                self.label_sensor_5["foreground"] = 'red'
        self.mqtt5.msg = None
        self.after(10, self.check_sensor_5)

    def check_sensor_6(self):
        if self.mqtt6.msg is not None:
            if self.mqtt6.msg == "c":
                self.label_sensor_6["foreground"] = 'green'
            elif self.mqtt6.msg == "w":
                self.label_sensor_6["foreground"] = 'blue'
            elif self.mqtt6.msg == "d":
                self.label_sensor_6["foreground"] = 'red'
        self.mqtt6.msg = None
        self.after(10, self.check_sensor_6)

    def start(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, START, qos=0)

    def stop(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, WAIT, qos=0)

    def close(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, STOP, qos=0)

    def stream_start(self):
        pass

    def stream_stop(self):
        pass


SensorControl()
