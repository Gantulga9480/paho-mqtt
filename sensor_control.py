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

        self.clients = list()
        self.kinects = list()
        self.sensor_state = list()

        self.is_streaming = False

        self.activity = StringVar()
        self.sensor_ignore = BooleanVar()
        self.buffer_ignore = BooleanVar()

        for item in KINECTS:
            self.kinects.append(Kinect(item[0], type_is=item[1]))

        # Clients
        for i, item in enumerate(SENSORS):
            self.clients.append(PahoMqtt(BROKER, f"sensor-{i+1}",
                                         c_msg=item))
            self.clients[i].subscribe(item)
            self.clients[i].loop_start()

        self.stream = Stream(sensor=self.clients, kinect=self.kinects)

        # Tk widgets
        self.title("Control")
        self.resizable(0, 0)
        self.configure(bg='white')

        # Sensor Frame 1
        self.sensor_frame1 = LabelFrame(self, text="Sensor control",
                                        background='white')
        self.sensor_frame1.pack(side=LEFT, fill="y")

        for i in range(len(SENSORS)):
            self.sensor_state.append(Label(self.sensor_frame1,
                                           text=f"SENSOR {i+1}",
                                           background='white',
                                           font=("default", 15, 'bold')))
            self.sensor_state[i].grid(row=i, column=0)

        for i in range(len(KINECTS)):
            self.sensor_state.append(Label(self.sensor_frame1,
                                           text=f"KINECT {i}",
                                           background='white',
                                           font=("default", 15, 'bold')))
            self.sensor_state[i+len(SENSORS)].grid(row=i+len(SENSORS),
                                                   column=0)

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

        self.stream_reset()

        self.stream_data()
        self.stream_video()

        self.set_state()
        self.refresh()

        # Main loop
        self.mainloop()
        # Do not write code after main loop

    def activity_start(self):
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        self.act_end_btn['state'] = NORMAL
        self.act_start_btn['state'] = DISABLED
        self.activity_list.append(self.activity.get())

        crnt_time = dt.today()
        self.time_start = time.time() - self.start_sec
        self.time_start_list.append(self.time_start)
        self.real_time_start.append(crnt_time)
        self.activity_time_list[0].append(len(self.sensor_stream))
        self.video_activity_time[0].append(len(self.video_stream[0]))

    def activity_end(self):
        self.stream_stop_btn['state'] = NORMAL
        self.stream_start_btn['state'] = DISABLED
        self.stream_save_btn['state'] = DISABLED
        self.stream_reset_btn['state'] = DISABLED
        self.act_end_btn['state'] = DISABLED
        self.act_start_btn['state'] = NORMAL

        crnt_time = dt.today()
        self.real_time_end.append(time.time() - self.start_sec)
        self.activity_time_list[1].append(len(self.sensor_stream))
        self.video_activity_time[1].append(len(self.video_stream[0]))
        print(self.video_activity_time[0][0])
        print(self.video_activity_time[1][0])

    def stream_start(self):
        sen_count = 0
        kin_count = 0
        for i in range(len(SENSORS)):
            if self.clients[i].sensor_ready:
                sen_count += 1
            else:
                if self.sensor_ignore:
                    sen_count += 1
                else:
                    messagebox.showwarning("Sensor Error",
                                           f"{SENSOR_ERROR}-{i+1}")
        for kinect in self.kinects:
            if kinect.is_ready():
                kin_count += 1
            else:
                messagebox.showwarning("Sensor Error",
                                       f"{KINECT_ERROR}-{kinect.id_name}")
                print(f"type: {kinect.type_is}, name: {kinect.id_name} error!")

        if sen_count == len(SENSORS) and kin_count == len(KINECTS):
            self.is_streaming = True
            for client in self.clients:
                client.is_streaming = True
            for kinect in self.kinects:
                kinect.is_streaming = True
        else:
            self.is_streaming = False
        if self.is_streaming:
            if len(self.sensor_stream) == 0:
                crnt_time = dt.today()
                self.start_time = crnt_time.strftime(TIME_FORMAT)
                self.date = crnt_time.strftime(DATE_FORMAT)
                self.data_time = crnt_time.strftime(DATE_TIME)

                self.time_path = \
                    f"data_by_time/{self.date}/{self.data_time}/"

                self.start_sec = time.time()
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
        crnt_time = dt.today()
        self.stop_sec = time.time()
        self.stop_time = crnt_time.strftime(TIME_FORMAT)
        self.stream_stop_btn['state'] = DISABLED
        self.stream_start_btn['state'] = NORMAL
        self.stream_save_btn['state'] = NORMAL
        self.stream_reset_btn['state'] = NORMAL
        self.act_end_btn['state'] = DISABLED
        self.act_start_btn['state'] = DISABLED

        self.is_streaming = False
        for client in self.clients:
            client.is_streaming = False
        for kinect in self.kinects:
            kinect.is_streaming = False

    def stream_save(self):
        os.makedirs(self.time_path)
        srt = open(f"{self.time_path}sub.srt", "w+")
        for index, label in enumerate(self.activity_list):
            self.sensor_stream[self.activity_time_list[0][index]-1][2] = \
                f"{label} start"
            self.sensor_stream[self.activity_time_list[1][index]-1][2] = \
                f"{label} end"
            act_frame = list()
            for i in range(self.activity_time_list[0][index]-1,
                           self.activity_time_list[1][index], 1):
                act_frame.append(self.sensor_stream[i])
            data_time = self.real_time_start[index].strftime(DATE_TIME)
            os.makedirs(f"activity/{label}/{data_time}")
            path = f"activity/{label}/{data_time}/sensor"
            data_open = open(f"{path}.csv", "w+", newline='')
            writer = csv.writer(data_open)
            with data_open:
                for row in act_frame:
                    writer.writerow(row)
            s_h, s_m, s_s = get_time(self.time_start_list[index], raw=True)
            t_h, t_m, t_s = get_time(self.real_time_end[index], raw=True)
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

            srt.writelines([f"{(index+1)*2-1}\n",
                            f"{s_h}:{s_m}:{s_s},000 --> {s_h_t}:{s_m_t}:{s_s_t},000\n",
                            f"{label} start\n",
                            "\n",
                            f"{(index+1)*2}\n",
                            f"{t_h}:{t_m}:{t_s},000 --> {t_h_t}:{t_m_t}:{t_s_t},000\n",
                            f"{label} end\n",
                            "\n"])
            xbox_rgb_out = cv2.VideoWriter(f"{path}_k1_rgb.avi",
                                           cv2.VideoWriter_fourcc(*'DIVX'),
                                           30, XBOX_KINECT_FRAME_SIZE)
            xbox_depth_out = cv2.VideoWriter(f"{path}_k1_depth.avi",
                                             cv2.VideoWriter_fourcc(*'DIVX'),
                                             30, XBOX_KINECT_FRAME_SIZE)
            azure_rgb_out = cv2.VideoWriter(f"{path}_k2_rgb.avi",
                                            cv2.VideoWriter_fourcc(*'DIVX'),
                                            30, AZURE_KINECT_RGB_SIZE)
            azure_depth_out = cv2.VideoWriter(f"{path}_k2_depth.avi",
                                              cv2.VideoWriter_fourcc(*'DIVX'),
                                              30, AZURE_KINECT_DEPTH_SIZE)
            rgb_frame = list()
            depth_frame = list()
            azure_rgb = list()
            azure_depth = list()
            for i in range(self.video_activity_time[0][index]-1,
                           self.video_activity_time[1][index], 1):
                rgb_frame.append(self.video_stream[0][i])
                depth_frame.append(self.depth_stream[0][i])
                azure_rgb.append(self.video_stream[1][i])
                azure_depth.append(self.depth_stream[1][i])

            for frame in rgb_frame:
                xbox_rgb_out.write(frame)
            for frame in depth_frame:
                xbox_depth_out.write(frame)
            for frame in azure_rgb:
                azure_rgb_out.write(frame)
            for frame in azure_depth:
                azure_depth_out.write(frame)
            xbox_rgb_out.release()
            xbox_depth_out.release()
            azure_rgb_out.release()
            azure_depth_out.release()
        srt.close()
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
        self.real_time_start = list()
        self.real_time_end = list()
        self.time_start_list = list()

        for _ in range(len(self.kinects)):
            self.video_stream.append([])
            self.depth_stream.append([])

        self.stream.video_buffer_empty_count = 0
        self.stream.sensor_buffer_empty_count = 0

        for sensor in self.clients:
            sensor.msg_buffer.clear()
        for kinect in self.kinects:
            kinect.rgb_buffer.clear()
            kinect.depth_buffer.clear()

    def stream_data(self):
        if self.is_streaming:
            try:
                data = self.stream.get_data()
                self.sensor_stream.append([dt.now(),
                                           data,
                                           0])
            except BufferError:
                messagebox.showerror("Error", BUFFER_ERROR)
                self.stream_stop()
                self.stream_reset()
        self.after(DATA_SPEED, self.stream_data)

    def stream_video(self):
        if self.is_streaming:
            try:
                video, depth = self.stream.get_video_stream()
                if video and depth:
                    for i in range(len(self.kinects)):
                        self.video_stream[i].append(video[i])
                        self.depth_stream[i].append(depth[i])
            except BufferError:
                messagebox.showerror("Error", BUFFER_ERROR)
                self.stream_stop()
                self.stream_reset()
        self.after(VIDEO_SPEED, self.stream_video)

    def summary(self):
        os.system("clear")
        print("----  RESULT SUMMARY  ----")
        print(f"start time: {self.start_time} ---> end time: {self.stop_time}")
        print(f"Duration: {get_time(self.stop_sec - self.start_sec)}")
        print("\n----------------------------------------------------")
        print("Activities performed:")
        for index, label in enumerate(self.activity_list):
            spaces = [" " for i in range(index)]
            space = "".join(spaces)
            print(f"{space}{index+1}: {label}")
        print("\n----------------------------------------------------")
        for kinect in self.kinects:
            print("kinect", kinect.id_name,
                  len(kinect.depth_buffer), "left in depth_buffer")
            print("kinect", kinect.id_name,
                  len(kinect.rgb_buffer), "left in rgb_buffer")
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
        for i in range(len(self.kinects)):
            if self.kinects[i].is_ready():
                self.sensor_state[i+len(SENSORS)]["foreground"] = 'green'
            else:
                self.sensor_state[i+len(SENSORS)]["foreground"] = 'red'
        self.after(1333, self.refresh)

    def set_state(self):
        for i in range(len(SENSORS)):
            self.clients[i].sensor_ready = False
        self.after(1003, self.set_state)


SensorControl()
