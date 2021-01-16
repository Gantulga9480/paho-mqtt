import paho.mqtt.client as mqtt
import time
import argparse
import numpy as np
import cv2
import pygame

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--sensor", type=int)
args = parser.parse_args()

SENSOR_MAX = 80
SENSOR_MIN = -20
SENSOR = args.sensor

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
clock = pygame.time.Clock()
font = pygame.font.SysFont("arial", 15)


def draw_grid(data):
    for i in range(8):
        for j in range(8):
            num = float(data[i][j])
            color = int((num-SENSOR_MIN)/(SENSOR_MAX-SENSOR_MIN)*255)
            pygame.draw.rect(win, (color, color, 255 - color),
                             (VEL*j, VEL*i, SHAPE, SHAPE))
            score = font.render(f"{num}", 1, WHITE)
            win.blit(score, (VEL*j, VEL*i))


def on_connect(client, userdata, level, buf):
    pass


def on_message(client, userdata, message):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()
    msg = message.payload.decode("utf-8", "ignore")
    msg = msg.replace("[", "")
    msg = msg.replace("]", "")
    data = msg.split(",")
    data = np.reshape(np.array(data), (8, 8))
    draw_grid(data)
    pygame.display.flip()
    clock.tick(30)


broker = "192.168.1.2"
client = mqtt.Client(f"sensor{SENSOR}_read")
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker)
time.sleep(0.1)
pygame.display.set_caption(f"Sensor {SENSOR}")
client.subscribe(f"sensors/sensor{SENSOR}_data", 2)
client.loop_forever()
time.sleep(0.1)
