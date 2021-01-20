"""git -> Gantulga9480"""

import time
import os
import numpy as np
import csv
import cv2
from pathlib import Path
from datetime import datetime as dt
from shutil import copyfile

from tkinter import *
from tkinter import ttk
from tkinter import messagebox

from app.user_info import UserInfo
from app.pop_up import PopUp
from app.paho_mqtt import PahoMqtt
from app.data_stream import Stream
from app.kinect import Kinect
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
        self.is_write = True
        self.stream_data = []
        self.stream_video = []
        self.stream_depth = []
        self.clients = []
        self.stream = None
        self.save_path = ""
        self.activity = StringVar()
        self.ignore = BooleanVar()
        self.k1 = Kinect(id=1)
        # self.k2 = Kinect(id=2)
        self.frame_size = (640, 480)

        # Clients
        for i in range(len(SENSORS)):
            self.clients.append(PahoMqtt(BROKER, f"sensor-{i}",
                                         c_msg=SENSORS[i]))
            self.clients[i].subscribe(SENSORS[i])
            self.clients[i].loop_start()

        # Tk widgets
        self.title("Control")
        self.resizable(0, 0)
        self.configure(bg='white')

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
                                    text="Refresh", command=self.refresh)
        self.start_btn.grid(row=8, column=0)

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
                                          value=ACTIVITIES,
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
        tool.add_checkbutton(label="Ignore sensor error",
                             onvalue=1, offvalue=0, variable=self.ignore)
        menubar.add_cascade(label="Tools", menu=tool)
        self.config(menu=menubar)

        self.save_data()
        self.save_video()
        self.set_state()
        self.refresh()

        # Main loop
        self.mainloop()
        # Do not write code after main loop

    def stream_start(self):
        self.stream_stop_btn['state'] = NORMAL
        self.stream_start_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = NORMAL
        self.stream = Stream(clients=self.clients, ignore=self.ignore.get())
        self.crnt_time = dt.today().strftime(FILEFORMAT)
        self.save_path = \
            f"data_by_activity/{self.activity.get()}/{self.crnt_time}/{self.crnt_time}"
        self.copy_path = \
            f"data_by_time/{self.crnt_time}/{self.crnt_time}"

        if Path(f"{self.save_path}.csv").is_file():
            is_write = messagebox.askyesno("Stream save",
                                           FILE_FOUND_MSG)
            if is_write:
                self.is_streaming = True
                self.k1.is_streaming = True
            else:
                pass
        else:
            self.is_streaming = True
            self.k1.is_streaming = True
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
        self.k1.is_streaming = False
        del(self.stream)

    def stream_save(self):
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        if len(self.stream_data) > 0:
            self.stream_data[1][2] = f"{self.activity.get()}-start"
            self.stream_data[-1][2] = f"{self.activity.get()}-end"
            os.makedirs(f"data_by_activity/{self.activity.get()}/{self.crnt_time}")
            os.makedirs(f"data_by_time/{self.crnt_time}")
            self.rgb_out = cv2.VideoWriter(f"{self.save_path}_rgb.avi",
                                           cv2.VideoWriter_fourcc(*'DIVX'),
                                           30, self.frame_size)
            self.depth_out = cv2.VideoWriter(f"{self.save_path}_depth.avi",
                                             cv2.VideoWriter_fourcc(*'DIVX'),
                                             30, self.frame_size)
            self.file_data = open(f"{self.save_path}.csv", "w+", newline='')
            writer = csv.writer(self.file_data)
            with self.file_data:
                for row in self.stream_data:
                    writer.writerow(row)
            for i in range(len(self.stream_video)):
                self.rgb_out.write(self.stream_video[i])
            for i in range(len(self.stream_depth)):
                self.depth_out.write(self.stream_depth[i])
            self.rgb_out.release()
            self.depth_out.release()
            copyfile(f"{self.save_path}.csv",
                     f"{self.copy_path}.csv")
            copyfile(f"{self.save_path}_rgb.avi",
                     f"{self.copy_path}_rgb.avi")
            copyfile(f"{self.save_path}_depth.avi",
                     f"{self.copy_path}_depth.avi")
        else:
            messagebox.showinfo("Stream save", "No Data to save!")
        print("left in depth_buffer", len(self.k1.depth_buffer))
        print("left in rgb_buffer", len(self.k1.rgb_buffer))
        self.stream_data.clear()
        self.stream_video.clear()
        self.stream_depth.clear()
        self.k1.depth_buffer.clear()
        self.k1.rgb_buffer.clear()

    def stream_reset(self):
        self.stream_reset_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = NORMAL
        self.stream_data.clear()
        self.stream_video.clear()
        self.stream_depth.clear()
        self.k1.depth_buffer.clear()
        self.k1.rgb_buffer.clear()
        for i in range(len(SENSORS)):
            self.clients[i].msg = None

    def save_data(self):
        if self.is_streaming:
            try:
                data = self.stream.get_data()
                self.stream_data.append([dt.now(),
                                         data,
                                         0])
            except Exception:
                messagebox.showwarning("Nothing to read", SENSOR_DATA_ERROR)
                self.is_streaming = False
                self.k1.is_streaming = True
                self.stream_reset()
        self.after(DATA_SPEED, self.save_data)

    def save_video(self):
        if self.k1.is_streaming:
            if len(self.k1.depth_buffer) > 0 and len(self.k1.rgb_buffer) > 0:
                depth = self.k1.depth_buffer[0]
                video = self.k1.rgb_buffer[0]
                self.k1.depth_buffer.pop(0)
                self.k1.rgb_buffer.pop(0)
                self.stream_video.append(video)
                self.stream_depth.append(depth)
        self.after(VIDEO_SPEED, self.save_video)

    def user_info(self):
        user = UserInfo(self)
        self.wait_window(user.win)
        self.age = user.age
        self.sex = user.sex
        self.weight = user.weight
        self.height = user.height
        self.age_label['text'] = user.age
        self.sex_label['text'] = user.sex
        self.height_label['text'] = user.height
        self.weight_label['text'] = user.weight
        del(user)

    def refresh(self):
        if self.clients[0].is_streaming:
            self.label_sensor_1["foreground"] = 'green'
        else:
            self.label_sensor_1["foreground"] = 'red'
        if self.clients[1].is_streaming:
            self.label_sensor_2["foreground"] = 'green'
        else:
            self.label_sensor_2["foreground"] = 'red'
        if self.clients[2].is_streaming:
            self.label_sensor_3["foreground"] = 'green'
        else:
            self.label_sensor_3["foreground"] = 'red'
        if self.clients[3].is_streaming:
            self.label_sensor_4["foreground"] = 'green'
        else:
            self.label_sensor_4["foreground"] = 'red'
        if self.clients[4].is_streaming:
            self.label_sensor_5["foreground"] = 'green'
        else:
            self.label_sensor_5["foreground"] = 'red'
        if self.clients[5].is_streaming:
            self.label_sensor_6["foreground"] = 'green'
        else:
            self.label_sensor_6["foreground"] = 'red'
        if self.k1.k1_depth_ready:
            self.label_sensor_7["foreground"] = 'green'
        else:
            self.label_sensor_7["foreground"] = 'red'
        """
        if self.k2.k2_depth_ready:
            self.label_sensor_8["foreground"] = 'green'
        else:
            self.label_sensor_8["foreground"] = 'red'
        """
        self.after(3333, self.refresh)

    def set_state(self):
        for i in range(6):
            self.clients[i].is_streaming = False
        self.after(1003, self.set_state)


SensorControl()
