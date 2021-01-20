import csv
import numpy as np
import argparse
import pygame
import tkinter as tk
from tkinter import filedialog

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--sensor", type=int)
args = parser.parse_args()

SENSOR = args.sensor

SENSOR_MAX = 40
SENSOR_MIN = 10

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 177, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)

WIDTH = 400
HEIGHT = 400
VEL = int(WIDTH / 8)
SHAPE = VEL - 1

root = tk.Tk()
root.filename = filedialog.askopenfilename(initialdir="/home/tulgaa/Desktop/net/paho-mqtt/",
                                           title="Select file",
                                           filetypes=(("csv file", "*.csv"),
                                                      ("all files", "*.*")))

pygame.init()


def draw_grid(data):
    for i in range(8):
        for j in range(8):
            num = float(data[i][j])
            color = int(np.floor((num-SENSOR_MIN)/(SENSOR_MAX-SENSOR_MIN)*255))
            if color > 255:
                color = 255
            if color < 0:
                color = 0
            pygame.draw.rect(win, (color, int(color/8), int((255 - color)/2)),
                             (VEL*j, VEL*i, SHAPE, SHAPE))
            score = font.render(f"{num}", 1, WHITE)
            win.blit(score, (VEL*j, VEL*i))


if len(root.filename) == 0:
    pass
else:
    file_1 = open(f"{root.filename}")
    data = []
    with file_1:
        csv_reader = csv.reader(file_1, delimiter=',')
        for i, row in enumerate(csv_reader):
            if i == 0:
                pass
            else:
                data.append(row[1].replace("[", "").replace("]", "").replace(" ", "").split(","))

    # data_1 = np.zeros((len(data), 64, 64))
    # data_2 = np.zeros((len(data), 64, 64))
    # data_3 = np.zeros((len(data), 64, 64))
    # data_4 = np.zeros((len(data), 64, 64))
    # data_5 = np.zeros((len(data), 64, 64))
    # data_6 = np.zeros((len(data), 64, 64))

    data_array = np.zeros((len(data), 64, 64))

    for i in range(len(data)):
        count = 64 * SENSOR - 64
        for j in range(8):
            for k in range(8):
                data_array[i][j][k] = data[i][count]
                count += 1

    win = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 15)
    pygame.display.set_caption("Sensor")
    run = True
    frame = 0
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        draw_grid(data_array[frame])
        pygame.display.flip()
        clock.tick(10)
        frame += 1
        if frame == len(data):
            frame = 0
