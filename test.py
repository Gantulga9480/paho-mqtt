import csv

file_ = open('file.csv', "w+", newline='')
writer = csv.writer(file_)
for i in range(10):
    writer.writerow([j for j in range(i)])
