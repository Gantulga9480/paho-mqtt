import csv
import numpy as np

file_1 = open("data_by_activity/activity_1/2021_01_19_03_13_00/2021_01_19_03_13_00.csv")
data = []
with file_1:
    csv_reader = csv.reader(file_1, delimiter=',')
    for i, row in enumerate(csv_reader):
        if i == 0:
            pass
        else:
            data = row[1].replace("[", "").replace("]", "").replace(" ", "").split(",")

data_1 = np.zeros((len(data), 64, 64))
for i in range(len(data)):
    for j in range(8):
        for k in range(8):
            data_1[i][j][k] = data[i*j]
