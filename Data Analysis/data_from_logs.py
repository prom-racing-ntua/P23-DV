import numpy as np
from matplotlib import pyplot as plt
import os
from math import *

paths = ["/home/nick/Downloads/run_46/canbus_controls_log.txt", "/home/nick/Downloads/run_46/canbus_sensor_log.txt"]
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
# print(np.shape(np.array(data)))
for i in range(len(data)):
    for j in range(len(data[i])):
        for k in range(len(data[i][j][0])):
            data[i][j][0][k] -= time_0
            data[i][j][0][k] *= 1e-3

selected = [[1], [2]]
legends = [['target'], ['actual']]
print(np.shape(np.array(data[1])))
for i in range(len(selected)):
    for k in range(len(selected[i])):
        j = selected[i][k]
        try:
            plt.plot(data[i][j][0], np.array(data[i][j][1])/max(data[i][j][1]), label =legends[i][k] )
        except:
            print(f'{i} {j}')
# plt.plot(t2, (np.diff(np.array(data[1][1][1]))/25e-3)/max(data[1][1][1]))
plt.legend()
plt.show()