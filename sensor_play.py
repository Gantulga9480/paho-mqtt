import pygame
import numpy as np
import csv
from cv2 import waitKey
# from threading import Thread
from threading import Thread
from tkinter import Tk, filedialog
import time
import test
from app.utils import Color as c


SENSORS = ['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5',
           'sensor_6']
SHAPE = 40
WIDTH = SHAPE*24
HEIGHT = SHAPE*16
FPS = 60
XY = [8, 8]

POSITION = [[SHAPE*16, SHAPE*8], [SHAPE*16, 0], [SHAPE*8, SHAPE*8],
            [SHAPE*8, 0], [0, SHAPE*8], [0, 0]]
SENSOR_MAX = 23
SENSOR_MIN = 19

tk_root = Tk()
tk_root.withdraw()
dir_name = filedialog.askdirectory()


class SensorData:

    def __init__(self, path, id) -> None:
        self.path = path
        self.data = []
        self.frame = None
        self.time_stamp = []
        self.run = True
        self.id = id

    def load(self):
        self.data.clear()
        file_1 = open(f"{self.path}")
        tmp = 0
        with file_1:
            csv_reader = csv.reader(file_1, delimiter=',')
            for i, row in enumerate(csv_reader):
                data = row[1].replace("[", "").replace("]", "")
                data = data.replace(" ", "").split(",")
                data = [float(item) for item in data]
                data = np.array(data).reshape(8, 8)
                data = np.flip(data, axis=1)
                date = row[0].split(' ')
                time_ = date[1].split(':')
                sec = int(time_[0]) * 3600 + \
                    int(time_[1]) * 60 + float(time_[2])
                if i == 0:
                    tmp = sec
                else:
                    dur = (sec - tmp)
                    # print(sec, tmp, dur)
                    tmp = sec
                    self.time_stamp.append(dur)
                self.data.append([data, row[2]])

    def replay(self):
        for i, frame in enumerate(self.data):
            if self.run:
                pass
            else:
                break
            self.frame = frame
            test.delayMicroseconds((self.time_stamp[i]-0.01)*10**6)


worker = []
sensors = []
drawers = []
for s in SENSORS:
    sensor = SensorData(dir_name + f'/{s}.csv', s)
    sensors.append(sensor)
    sensor.load()
    print(s)
    worker.append(Thread(target=sensor.replay))


pygame.init()
clock = pygame.time.Clock()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Sensor')
font = pygame.font.SysFont("arial", 20)
run = True


def draw(pos, sensor):
    pos = POSITION[pos]
    while run:
        data = sensor.frame[0]
        label = sensor.frame[1]
        if label != '0':
            print(label)
        for i in range(XY[0]):
            if run:
                pass
            else:
                break
            for j in range(XY[1]):
                if run:
                    pass
                else:
                    break
                val = data[i, j]
                color = int((val-SENSOR_MIN)/(SENSOR_MAX-SENSOR_MIN)*255)
                if color > 255:
                    color = 255
                if color < 0:
                    color = 0
                # print((color, 64, (255 - color)))
                pygame.draw.rect(win, (color, 64, (255 - color)),
                                 (SHAPE*j+pos[0], SHAPE*i + pos[1],
                                  SHAPE, SHAPE))
                # score = font.render(f"{}", 1, c.WHITE)
                # win.blit(score, (SHAPE*j+pos[0], SHAPE*i+pos[1]))
                lbl = font.render(sensor.id, 1, c.BLACK)
                win.blit(lbl, (pos[0]+SHAPE*4, pos[1]+SHAPE*4))
                pygame.display.flip()


for i in range(len(SENSORS)):
    drawers.append(Thread(target=draw, args=(i, sensors[i])))
for i, thread in enumerate(worker):
    thread.start()
    drawers[i].start()
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            for s in sensors:
                s.run = False
            run = False
