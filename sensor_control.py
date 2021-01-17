"""
All paths added to app/utils.py
All commands added to app/utils.py
All clients added to app/utils.py
All activities added to app/utils.py
"""

import time
import os
import numpy as np
import csv
from pathlib import Path
from datetime import datetime as dt
from tkinter import *
from tkinter import ttk
from tkinter import messagebox 
import app.user_info as ui
from app.pop_up import PopUp
from app.paho_mqtt import PahoMqtt
from app.data_stream import Stream
from app.utils import *


class SensorControl(Tk):

    def __init__(self, screenName=None, baseName=None,
                 useTk=1, sync=0, use=None):
        super().__init__(screenName=screenName, baseName=baseName,
                         useTk=useTk, sync=sync, use=use)

        # Attributes
        self.age = "User"
        self.sex = "User"
        self.height = "User"
        self.weight = "User"
        self.is_streaming = False
        self.stream_data = []
        self.stream = None
        self.options = ACTIVITIES
        self.activity = StringVar()
        self.save_path = ""
        self.is_write = True

        print(self.options)

        # Clients
        self.mqtt = PahoMqtt(BROKER, "master")
        self.mqtt.loop_start()

        self.mqtt1 = PahoMqtt(BROKER, "s1")
        self.mqtt1.subscribe(CLEINT_1)
        self.mqtt1.loop_start()

        self.mqtt2 = PahoMqtt(BROKER, "s2")
        self.mqtt2.subscribe(CLEINT_2)
        self.mqtt2.loop_start()

        self.mqtt3 = PahoMqtt(BROKER, "s3")
        self.mqtt3.subscribe(CLEINT_3)
        self.mqtt3.loop_start()

        self.mqtt4 = PahoMqtt(BROKER, "s4")
        self.mqtt4.subscribe(CLEINT_4)
        self.mqtt4.loop_start()

        self.mqtt5 = PahoMqtt(BROKER, "s5")
        self.mqtt5.subscribe(CLEINT_5)
        self.mqtt5.loop_start()

        self.mqtt6 = PahoMqtt(BROKER, "s6")
        self.mqtt6.subscribe(CLEINT_6)
        self.mqtt6.loop_start()

        self.mqttd1 = PahoMqtt(BROKER, "d1")
        self.mqttd1.subscribe(SENSOR_1)
        self.mqttd1.loop_start()

        self.mqttd2 = PahoMqtt(BROKER, "d2")
        self.mqttd2.subscribe(SENSOR_2)
        self.mqttd2.loop_start()

        self.mqttd3 = PahoMqtt(BROKER, "d3")
        self.mqttd3.subscribe(SENSOR_3)
        self.mqttd3.loop_start()

        self.mqttd4 = PahoMqtt(BROKER, "d4")
        self.mqttd4.subscribe(SENSOR_4)
        self.mqttd4.loop_start()

        self.mqttd5 = PahoMqtt(BROKER, "d5")
        self.mqttd5.subscribe(SENSOR_5)
        self.mqttd5.loop_start()

        self.mqttd6 = PahoMqtt(BROKER, "d6")
        self.mqttd6.subscribe(SENSOR_6)
        self.mqttd6.loop_start()

        # Tk widgets
        self.title("Control")
        self.resizable(0,0)
        self.configure(bg='white')

        s = ttk.Style()
        s.configure("Red.TLabel", foreground='red')
        s.configure("Green.TLabel", foreground='green')
        s.configure("Black.TLabel", foreground='black')
        s.configure("Yellow.TLabel", foreground='yellow')
        s.configure('White.TLabelFrame', background='white')

        # Sensor Frame 1
        self.sensor_frame1 = LabelFrame(self, text="Sensor control",
                                        background='white')
        self.sensor_frame1.pack(side=LEFT, fill="y")

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
                                    text="Start", command=self.sensor_start)
        self.start_btn.grid(row=8, column=0)
        self.stop_btn = ttk.Button(self.sensor_frame1,
                                   text="Stop", command=self.sensor_stop)
        self.stop_btn.grid(row=8, column=1)

        # Stream Frame 2
        self.sensor_frame2 = LabelFrame(self, text="Data control",
                                        background='white')
        self.sensor_frame2.pack(side=LEFT, fill="y")
        self.user_age = Label(self.sensor_frame2, text="Age",
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.user_age.grid(row=0, column=0)
        self.age_label = Label(self.sensor_frame2, text=self.age,
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.age_label.grid(row=0, column=1)
        self.user_sex = Label(self.sensor_frame2, text="Sex",
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.user_sex.grid(row=1, column=0)
        self.sex_label = Label(self.sensor_frame2, text=self.sex,
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.sex_label.grid(row=1, column=1)
        self.user_height = Label(self.sensor_frame2, text="Height",
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.user_height.grid(row=2, column=0)
        self.height_label = Label(self.sensor_frame2, text=self.height,
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.height_label.grid(row=2, column=1)
        self.user_weight = Label(self.sensor_frame2, text="Weight",
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.user_weight.grid(row=3, column=0)
        self.weight_label = Label(self.sensor_frame2, text=self.weight,
                                    background='white',
                                    font=("default", 10, 'bold'))
        self.weight_label.grid(row=3, column=1)

        self.activity_menu = ttk.Combobox(self.sensor_frame2,
                                          value=self.options,
                                          textvariable=self.activity)
        self.activity_menu.current(0)
        self.activity_menu.config(state="readonly", width=10)
        self.activity_menu.bind("<<ComboboxSelected>>")
        self.activity_menu.grid(row=4, column=0, columnspan=2)
        
        self.stream_start_btn = ttk.Button(self.sensor_frame2,
                                                text="Steam start",
                                                command=self.stream_start)
        self.stream_start_btn.grid(row=5, column=0)
        self.stream_stop_btn = ttk.Button(self.sensor_frame2,
                                               text="Steam stop",
                                               command=self.stream_stop)
        self.stream_stop_btn["state"] = DISABLED
        self.stream_stop_btn.grid(row=5, column=1)

        self.stream_reset_btn = ttk.Button(self.sensor_frame2,
                                               text="Stream reset",
                                               command=self.stream_reset)
        self.stream_reset_btn["state"] = DISABLED
        self.stream_reset_btn.grid(row=6, column=1)

        self.stream_save_btn = ttk.Button(self.sensor_frame2,
                                               text="Stream save",
                                               command=self.stream_save)
        self.stream_save_btn["state"] = DISABLED
        self.stream_save_btn.grid(row=6, column=0)

        # Menu
        menubar = Menu(self)
        tool = Menu(menubar, tearoff=0)
        tool.add_command(label="Insert user info", command=self.user_info)
        tool.add_separator()
        tool.add_command(label="Disconnect sensors", command=self.sensor_close)
        menubar.add_cascade(label="Tools", menu=tool)
        self.config(menu=menubar)

        # Check sensors
        self.check_sensor_1()
        self.check_sensor_2()
        self.check_sensor_3()
        self.check_sensor_4()
        self.check_sensor_5()
        self.check_sensor_6()

        self.save_data()
        
        # Main loop
        # Code lines after this function wont run
        self.mainloop()

    def sensor_start(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, START_COMMAND, qos=0)

    def sensor_stop(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, WAIT_COMMAND, qos=0)

    def sensor_close(self):
        self.mqtt.publish(PC_SENSOR_CONTROL, STOP_COMMAND, qos=0)

    def stream_start(self):
        self.stream_stop_btn['state'] = NORMAL
        self.stream_start_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = NORMAL
        self.stream = Stream(clients=[self.mqttd2])
        self.save_path = \
            f"data/{self.activity.get()}/{dt.today().strftime(FILEFORMAT)}.csv"
        if Path(self.save_path).is_file():
            is_write = messagebox.askyesno("Stream save",
                                            FILE_FOUND_MSG)
            if is_write:
                self.is_streaming = True
            else:
                pass
        else:
            self.is_streaming = True
        if self.is_streaming:
            if len(self.stream_data) == 0:
                self.stream_data.append([self.age, self.sex,
                                         self.height, self.weight])

    def stream_stop(self):
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = NORMAL
        if len(self.stream_data) > 0:
            self.stream_save_btn['state'] = NORMAL
            self.stream_reset_btn['state'] = NORMAL
        else:
            pass
        self.is_streaming = False

    def stream_save(self):
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        if len(self.stream_data) > 0:
            self.file_data = open(self.save_path, "w+", newline ='')
            writer = csv.writer(self.file_data)
            with self.file_data:
                for row in self.stream_data:
                    writer.writerow(row)
        else:
            messagebox.showinfo("Stream save", "No Data to save!")
        self.stream_data.clear()

    def save_data(self):
        if self.is_streaming:
            data = self.stream.get_data()
            print(data)
            self.stream_data.append([dt.now(),
                                     data,
                                     self.activity.get()])
        self.after(SPEED, self.save_data)

    def stream_reset(self):
        self.stream_reset_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_data.clear()

    def user_info(self):
        user = ui.UserInfo(self)
        self.wait_window(user.win)
        self.age = user.age
        self.sex = user.sex
        self.weight = user.weight
        self.height = user.height
        self.age_label['text'] = user.age
        self.sex_label['text'] = user.sex
        self.height_label['text'] = user.height
        self.weight_label['text'] = user.weight

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


SensorControl()
