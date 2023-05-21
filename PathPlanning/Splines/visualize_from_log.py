import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

script_path = os.path.realpath(__file__)
log_file_path = os.path.abspath(os.path.join(script_path, os.pardir, "spline_log.txt"))
log_file = open(log_file_path, 'r')

index = 1
spline_points = np.empty((1,2))

def animate(i):
    global index, spline_points, log_file
    line = log_file.readline()
    nums = [float(a) for a in line.strip().split(' ') if a != '']
    if nums[0] == index:
        index += 1
        print("New Spline")
        print(spline_points[1:,:])

        data = spline_points[1:,:]
        ax.clear()
        ax.plot(spline_points[:,0], spline_points[:,1])
        ax.set_ylim([-30,30])
        ax.set_xlim([-100,100])

        spline_points = np.empty((1,2))
    else:
        new_row = np.array(nums)
        spline_points = np.vstack((spline_points, new_row))

fig,ax = plt.subplots()
ani = FuncAnimation(fig, animate, interval=1, repeat=False)
plt.show()