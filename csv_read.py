import csv

file_1 = open("data/activity_1/test.csv")

with file_1:
    csv_reader = csv.reader(file_1, delimiter=',')
    for row in csv_reader:
        print(row[1])