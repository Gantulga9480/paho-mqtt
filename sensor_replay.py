import numpy as np
import pygame
import csv


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
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"Sensor")
clock = pygame.time.Clock()
font = pygame.font.SysFont("arial", 15)

file_1 = open("data_by_activity/activity_1/2021_01_19_03_13_00/2021_01_19_03_13_00.csv")
data = []

with file_1:
    csv_reader = csv.reader(file_1, delimiter=',')
    for i, row in enumerate(csv_reader):
        if i == 0:
            pass
        else:
            data.append(row[0])



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

while True:
    pass

