import numpy as np
from matplotlib import pyplot as plt
import os
from math import *

paths = ["/home/nick/Desktop/timestamp_logs/run_5/canbus_sensor_log.txt", "/home/nick/Desktop/timestamp_logs/run_5/vn300_log.txt"]
data = []
time_z = []
# file
# -> datapoints
# -> -> time -> data 

for path in paths:
    file = open(path, 'r')
    lines = file.readlines()
    for i in range(len(lines)): lines[i] = lines[i].split()
    time_z.append(float(lines[0][0]))
    data.append([])
    for j in range(len(lines[0])-3):
        data[-1].append([[],[]])     
        for line in lines:
            data[-1][-1][0].append(float(line[0]))
            data[-1][-1][1].append(float(line[j+3]))

time_0 = min(time_z)
print(time_0)
# print(np.shape(np.array(data)))
for i in range(len(data)):
    for j in range(len(data[i])):
        for k in range(len(data[i][j][0])):
            data[i][j][0][k] -= time_0
            data[i][j][0][k] *= 1e-3
path_sel = [1]
dp_sel = [0, 1, 2]
# a_s = [[], [], []]

# for x in data[0][0][1]: a_s[0].append(x)
# for x in data[0][1][1]: a_s[1].append(x)
# for x in data[0][2][1]: a_s[2].append(x)
# a_x = -np.average(a_s[0])
# a_y = -np.average(a_s[1])
# a_z = -np.average(a_s[2])

# roll = atan2(a_y, a_z)
# pitch = atan2(-a_x, sqrt(a_y**2 + a_z**2))

# v = data[1][1][1]
# v_ = [0]
# v_.append(data[1][1][1][:-2])
# plt.plot(data[1][1][0], (np.array(v_)-np.array(v))/25e-3)
t2 = data[1][1][0][:-1]
for i in path_sel:
    for j in dp_sel:
        plt.plot(data[i][j][0], np.array(data[i][j][1])/max(data[i][j][1]))
# plt.plot(t2, (np.diff(np.array(data[1][1][1]))/25e-3)/max(data[1][1][1]))
plt.show()