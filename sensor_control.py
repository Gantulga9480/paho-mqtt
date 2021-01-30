"""git -> Gantulga9480"""

import time
import os
import numpy as np
import csv
import cv2

from pathlib import Path
from datetime import datetime as dt
from shutil import copyfile
import pyk4a
from pyk4a import ColorResolution
from pyk4a import Config, PyK4A

from tkinter import Tk
from tkinter import Label
from tkinter import ttk
from tkinter import messagebox
from tkinter import StringVar
from tkinter import BooleanVar
from tkinter import LabelFrame
from tkinter import Menu
from tkinter import DISABLED
from tkinter import LEFT
from tkinter import NORMAL

from app.user_info import UserInfo
from app.pop_up import PopUp
from app.paho_mqtt import PahoMqtt
from app.data_stream import Stream

from app.utils import *


class SensorControl(Tk):

    def __init__(self, screenName=None, baseName=None,
                 useTk=1, sync=0, use=None):
        super().__init__(screenName=screenName, baseName=baseName,
                         useTk=useTk, sync=sync, use=use)

        #######################################################################
        # Attributes ----------------------------------------------------------
        self.age = "User"
        self.sex = "User"
        self.height = "User"
        self.weight = "User"

        self.is_activity_started = False
        self.is_streaming = False
        self.time_2 = 0
        self.time_1 = 0

        self.frame_count = 0

        self.activity = StringVar()
        self.sensor_ignore = BooleanVar()
        self.buffer_ignore = BooleanVar()

        # Clients
        self.kinect_client = PahoMqtt(BROKER, "KINECT control", c_msg="kinect")
        self.kinect_client.loop_start()
        self.clients = list()
        for i, item in enumerate(SENSORS):
            if item[2]:
                self.clients.append(PahoMqtt(BROKER, f"SENSOR {item[1]}",
                                             c_msg=item[0]))
                self.clients[i].subscribe(item[0])
                self.clients[i].loop_start()
            else:
                pass

        # sensor stream
        self.stream = Stream(sensor=self.clients)

        # Attributes ----------------------------------------------------------
        #######################################################################
        # Tk widgets ----------------------------------------------------------

        self.title("Control")
        self.resizable(0, 0)
        self.configure(bg='white')

        # Sensor Frame 1
        self.sensor_frame1 = LabelFrame(self, text="Sensor control",
                                        background='white')
        self.sensor_frame1.pack(side=LEFT, fill="y")

        self.sensor_state = list()
        for item in self.clients:
            self.sensor_state.append(Label(self.sensor_frame1,
                                           text=f"{item.info}",
                                           background='white',
                                           font=("default", 15, 'bold')))
            self.sensor_state[-1].grid(row=len(self.sensor_state),
                                       column=0)

        self.start_btn = ttk.Button(self.sensor_frame1,
                                    text="Refresh", command=self.refresh)
        self.start_btn.grid(row=len(self.clients)+1, column=0)

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
        self.activity_menu.config(state="readonly", width=15)
        self.activity_menu.bind("<<ComboboxSelected>>")
        self.activity_menu.grid(row=4, column=0, columnspan=2, pady=5)

        self.stream_start_btn = ttk.Button(self.sensor_frame2,
                                           text="Stream start",
                                           command=self.stream_start,
                                           width=11)
        self.stream_start_btn.grid(row=5, column=0, padx=2, pady=2)
        self.stream_stop_btn = ttk.Button(self.sensor_frame2,
                                          text="Stream stop",
                                          command=self.stream_stop,
                                          width=11)
        self.stream_stop_btn["state"] = DISABLED
        self.stream_stop_btn.grid(row=5, column=1, padx=2, pady=2)

        self.stream_reset_btn = ttk.Button(self.sensor_frame2,
                                           text="Stream reset",
                                           command=self.stream_reset,
                                           width=11)
        self.stream_reset_btn["state"] = DISABLED
        self.stream_reset_btn.grid(row=7, column=1, padx=2, pady=2)

        self.stream_save_btn = ttk.Button(self.sensor_frame2,
                                          text="Stream save",
                                          command=self.stream_save,
                                          width=11)
        self.stream_save_btn["state"] = DISABLED
        self.stream_save_btn.grid(row=7, column=0, padx=2, pady=2)

        self.act_start_btn = ttk.Button(self.sensor_frame2,
                                        text="Activity start",
                                        command=self.activity_start,
                                        width=11)
        self.act_start_btn["state"] = DISABLED
        self.act_start_btn.grid(row=6, column=0, padx=2, pady=5)

        self.act_end_btn = ttk.Button(self.sensor_frame2,
                                      text="Activity end",
                                      command=self.activity_end,
                                      width=11)
        self.act_end_btn["state"] = DISABLED
        self.act_end_btn.grid(row=6, column=1, padx=2, pady=5)

        # Menu
        menubar = Menu(self)
        tool = Menu(menubar, tearoff=0)
        tool.add_command(label="Insert user info", command=self.user_info)
        tool.add_checkbutton(label="Ignore sensor error",
                             onvalue=1, offvalue=0,
                             variable=self.sensor_ignore)
        tool.add_checkbutton(label="Ignore buffer error",
                             onvalue=1, offvalue=0,
                             variable=self.buffer_ignore)
        menubar.add_cascade(label="Tools", menu=tool)
        self.config(menu=menubar)

        # Tk widgets ----------------------------------------------------------
        #######################################################################
        # Sensor controls -----------------------------------------------------

        self.azure = PyK4A()
        self.azure.start()

        self.stream_reset()
        self.stream_sensor()
        self.get_video()
        self.stream_video()
        self.stream_depth()

        self.set_state()
        self.refresh()

        # Main loop -----------------------------------------------------------
        self.mainloop()
        # Main loop -----------------------------------------------------------
        #######################################################################

    def stream_sensor(self):
        if self.is_streaming:
            try:
                data, is_full = self.stream.get_data()
                if is_full:
                    self.sensor_stream.append([dt.now(),
                                               data,
                                               0])
                else:
                    pass
            except BufferError:
                messagebox.showerror("Error", BUFFER_ERROR)
                self.stream_stop()
                self.stream_reset()
        self.after(DATA_SPEED, self.stream_sensor)

    def get_video(self):
        if self.is_streaming:
            # self.time_1 = time.perf_counter()
            # print("time-1", self.time_1)
            img = self.azure.get_capture()
            # print(round(self.time_2 - self.time_1, 6))
            # img_color = img.color
            # img_color = img_color[:, :, 2::-1].astype(np.uint8)
            # img_color = img_color[:, :, 2::-1]
            # cv2.imshow("color", img_color)
            # cv2.waitKey(10)
            if np.any(img.color):
                self.video_stream.append(img.color)
                self.depth_stream.append(img.depth)
                # pass
            else:
                pass
            # self.time_2 = time.perf_counter()
            # print("time-2", self.time_2)
            # print(round(self.time_2 - self.time_1, 6))
        self.after(30 - int((self.time_2 - self.time_1)*1000), self.get_video)
        # self.after(15, self.get_video)

    def stream_video(self):
        if self.is_streaming:
            if self.video_stream.__len__() > 0:
                self.frame_count += 1
                img_color = self.video_stream.pop(0)
                img_color = img_color[:, :, 2::-1].astype(np.uint8)
                img_color = img_color[:, :, 2::-1]
                self.rgb_out.write(img_color)
            elif self.video_stream.__len__() == 0:
                pass
        self.after(VIDEO_SPEED, self.stream_video)

    def stream_depth(self):
        if self.is_streaming:
            if self.depth_stream.__len__() > 0:
                img_depth = self.depth_stream.pop(0)
                cv2.normalize(img_depth, img_depth, 0,
                              255, cv2.NORM_MINMAX)
                img_depth = cv2.cvtColor(img_depth,
                                         cv2.COLOR_GRAY2RGB)
                img_depth = img_depth.astype(np.uint8)
                self.depth_out.write(img_depth)
            elif self.depth_stream.__len__() == 0:
                pass
        self.after(VIDEO_SPEED, self.stream_depth)

    def activity_start(self):
        self.is_activity_started = True
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        self.act_end_btn['state'] = NORMAL
        self.act_start_btn['state'] = DISABLED

        self.activity_list.append(self.activity.get())
        data_time = dt.today().strftime(DATE_TIME)
        self.activity_time_list[0].append(len(self.sensor_stream))
        self.video_activity_time[0].append(self.frame_count)
        self.path = f"activity/{self.activity.get()}/{data_time}"

    def activity_end(self):
        self.is_activity_started = False

        self.stream_stop_btn['state'] = NORMAL
        self.stream_start_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        self.act_end_btn['state'] = DISABLED
        self.act_start_btn['state'] = NORMAL

        self.activity_time_list[1].append(len(self.sensor_stream))
        self.video_activity_time[1].append(self.frame_count)

    def stream_start(self):
        sen_count = 0
        for i in range(len(self.clients)):
            if self.clients[i].sensor_ready:
                sen_count += 1
            else:
                if self.sensor_ignore.get():
                    sen_count += 1
                else:
                    messagebox.showwarning("Sensor Error",
                                           f"{SENSOR_ERROR}-{i+1}")

        if sen_count == len(self.clients):
            self.is_streaming = True
            for client in self.clients:
                client.is_streaming = True
        else:
            self.is_streaming = False
        if self.is_streaming:
            if len(self.sensor_stream) == 0:
                crnt_time = dt.today()
                self.start_time = crnt_time.strftime(TIME_FORMAT)
                self.date = crnt_time.strftime(DATE_FORMAT)
                self.data_time = crnt_time.strftime(DATE_TIME)

                self.time_path = \
                    f"data/{self.date}/{self.data_time}"
                msg = f'start-{self.time_path}'
                self.kinect_client.publish(topic='kinect1', msg=msg, qos=0)
                os.makedirs(self.time_path)
                self.srt = open(f"{self.time_path}/k2_rgb.srt", "w+")

                self.rgb_out = cv2.VideoWriter(f"{self.time_path}/k2_rgb.avi",
                                               cv2.VideoWriter_fourcc(*'DIVX'),
                                               FPS, AZURE_KINECT_RGB_SIZE)
                self.depth_out = cv2.VideoWriter(f"{self.time_path}/k2_depth.avi",
                                                 cv2.VideoWriter_fourcc(*'DIVX'),
                                                 FPS, AZURE_KINECT_DEPTH_SIZE)

                self.start_sec = dt.today()
                self.stream.set_error(self.sensor_ignore.get(),
                                      self.buffer_ignore.get())

            self.stream_start_btn['state'] = DISABLED
            self.stream_start_btn['text'] = "Resume stream"
            self.stream_stop_btn['state'] = NORMAL
            self.stream_reset_btn['state'] = DISABLED
            self.stream_save_btn['state'] = DISABLED
            self.act_end_btn['state'] = DISABLED
            self.act_start_btn['state'] = NORMAL
        else:
            self.stream_stop()
            self.stream_reset()

    def stream_stop(self):
        self.is_streaming = False
        for client in self.clients:
            client.is_streaming = False
        crnt_time = dt.today()
        self.stop_sec = crnt_time
        self.stop_time = crnt_time.strftime(TIME_FORMAT)

        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = NORMAL
        self.stream_save_btn['state'] = NORMAL
        self.stream_reset_btn['state'] = NORMAL
        self.act_end_btn['state'] = DISABLED
        self.act_start_btn['state'] = DISABLED
        try:
            self.rgb_out.release()
            self.depth_out.release()
            del(self.rgb_out)
            del(self.depth_out)
        except AttributeError:
            pass

    def stream_save(self):
        for index, label in enumerate(self.activity_list):
            self.sensor_stream[self.activity_time_list[0][index]-1][2] = \
                f"{label} start"
            self.sensor_stream[self.activity_time_list[1][index]-1][2] = \
                f"{label} end"
            # srt file write
            time_start = self.video_activity_time[0][index] * 33
            time_stop = self.video_activity_time[1][index] * 33
            s_h, s_m, s_s, ms = get_time_1(time_start)
            t_h, t_m, t_s, t_ms = get_time_1(time_stop)
            s_s_t = str((s_s + SUB_DURATION) % 60).zfill(2)
            s_m_t = str((s_m + (s_s + SUB_DURATION)//60)).zfill(2)
            s_h_t = str((s_h + ((s_m + (s_s +
                                        SUB_DURATION)//60)//60))).zfill(2)

            t_s_t = str((t_s + SUB_DURATION) % 60).zfill(2)
            t_m_t = str((t_m + (t_s + SUB_DURATION)//60)).zfill(2)
            t_h_t = str((t_h + ((t_m + (t_s +
                                        SUB_DURATION)//60)//60))).zfill(2)
            s_h = str(s_h).zfill(2)
            s_m = str(s_m).zfill(2)
            s_s = str(s_s).zfill(2)
            t_h = str(t_h).zfill(2)
            t_m = str(t_m).zfill(2)
            t_s = str(t_s).zfill(2)
            self.srt.writelines([f"{(index+1)*2-1}\n",
                                 f"{s_h}:{s_m}:{s_s},{ms} --> {s_h_t}:{s_m_t}:{s_s_t},{ms}\n",
                                 f"{label} start\n",
                                 "\n",
                                 f"{(index+1)*2}\n",
                                 f"{t_h}:{t_m}:{t_s},{t_ms} --> {t_h_t}:{t_m_t}:{t_s_t},{t_ms}\n",
                                 f"{label} end\n",
                                 "\n"])
        self.srt.close()
        csv_file = open(f'{self.time_path}/sensor_data.csv', "w+", newline='')
        writer = csv.writer(csv_file)
        with csv_file:
            for item in self.sensor_stream:
                writer.writerow(item)
        self.summary()
        self.stream_reset()

    def stream_reset(self):
        self.stream_start_btn['text'] = "Stream start"
        self.stream_reset_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = NORMAL
        self.act_start_btn['state'] = DISABLED
        self.act_start_btn['state'] = DISABLED
        self.sensor_stream = list()
        self.video_stream = list()
        self.depth_stream = list()
        self.activity_list = list()
        self.activity_time_list = [[], []]
        self.video_activity_time = [[], []]

        for sensor in self.clients:
            sensor.msg_buffer.clear()

    def summary(self):
        os.system("clear")
        print("----  RESULT SUMMARY  ----")
        print(f"start time: {self.start_time} ---> end time: {self.stop_time}")
        print(f"Duration: {get_time_date(self.start_sec, self.stop_sec)}")
        print("\n----------------------------------------------------")
        print("Activities performed:")
        for index, label in enumerate(self.activity_list):
            spaces = [" " for i in range(index)]
            space = "".join(spaces)
            print(f"{space}{index+1}: {label}")
        print("\n----------------------------------------------------")
        for sensor in self.clients:
            print(sensor.info, len(sensor.msg_buffer), "left in msg_buffer")
        print("----------------------------------------------------")

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
        for i in range(len(self.clients)):
            if self.clients[i].sensor_ready:
                self.sensor_state[i]["foreground"] = 'green'
            else:
                self.sensor_state[i]["foreground"] = 'red'

    def set_state(self):
        for i in range(len(self.clients)):
            self.clients[i].sensor_ready = False
        self.after(1000, self.set_state)


SensorControl()
