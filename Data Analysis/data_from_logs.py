import numpy as np
from matplotlib import pyplot as plt
import os
from math import *

def log_from_run(run):
    paths = [f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/canbus_sensor_log.txt",
             f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/canbus_controls_log.txt", 
             f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/canbus_velocity_log.txt",
             f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/canbus_steering_log.txt",
             f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/vn300_log.txt",
             f"/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2/run_{run}/vn200_log.txt"
             ]
    data = []
    time_z = []
    # file
    # -> datapoints
    # -> -> time -> data 

    for path in paths:
        with open(path, 'r') as file:
            lines = file.readlines()
            for i in range(len(lines)): lines[i] = lines[i].split()
            time_z.append(float(lines[0][0]))
            data.append([])
            for j in range(len(lines[0])-3):
                data[-1].append([[],[]])     
                for line in lines: 
                    try:
                        if(int(line[1])==0):
                            data[-1][-1][0].append(float(line[0]))
                            data[-1][-1][1].append(float(line[j+3]))
                    except:
                        continue


    time_0 = min(time_z)

    # vt0 = time_z[4] - time_0
    # for i in range(len(data[4][0][0])):
    #     for j in range(len(data[4])):
    #         data[4][j][0][i] -= vt0

    # vt0 = time_z[5] - time_0
    # for i in range(len(data[5][0][0])):
    #     for j in range(len(data[5])):
    #         data[5][j][0][i] -= vt0


    # print(np.shape(np.array(data)))
    for i in range(len(data)):
        for j in range(len(data[i])):
            for k in range(len(data[i][j][0])):
                data[i][j][0][k] -= time_0
                data[i][j][0][k] *= 1e-3


    roll = np.deg2rad(2.181913870022126 )
    pitch =  np.deg2rad(13.00079787866103)
    yaw = 0.0
    def eul2rotm(yaw, pitch, roll):
        yaw_rotation = np.array([[ cos(yaw), -sin(yaw), 0],
                                [ sin(yaw),  cos(yaw), 0],
                                [        0,         0, 1]], dtype=np.float64)

        pitch_rotation = np.array([[ cos(pitch), 0, sin(pitch)],
                                [          0, 1,          0],
                                [-sin(pitch), 0, cos(pitch)]], dtype=np.float64)

        roll_rotation = np.array([[ 1,         0,         0],
                                [ 0, cos(roll),-sin(roll)],
                                [ 0, sin(roll), cos(roll)]], dtype=np.float64)
        return yaw_rotation @ pitch_rotation @ roll_rotation

    # vx = np.array(data[1][3][1])
    # vy = np.array(data[1][4][1])
    # vz = np.array(data[1][5][1])

    # v = np.array([vx, vy, vz])
    # print(np.shape(v))
    # vc = rotation_all @ v
    # data[0][1][1] = np.array(data[0][1][1])/3.9


    # vx_kalman = data[1][0][1]
    # vx_300 = data[2][3][1]
    # ax_200 = data[3][0][1]
    # ay_200 = data[3][1][1]
    # az_200 = data[3][2][1]
    # a_200 = [ax_200, ay_200, az_200]

    # a_x = np.mean(ax_200[1:400])
    # a_y = np.mean(ay_200[1:400])
    # a_z = np.mean(az_200[1:400])
    # roll = atan2(-a_y, -a_z)
    # pitch = atan2(a_x, sqrt(a_y**2 + a_z**2))
    # rot = eul2rotm(0, pitch, roll)
    # ac_200 = rot @ a_200
    # print(roll * 180 / pi, pitch * 180 / pi)

    # ax_200, ay_200, az_200 = ac_200

    # vx_integ = np.zeros((len(ax_200),))
    # for i in range(1, len(ax_200)):
    #     # print(vx_300[i-1], ax_200[i])
    #     vx_integ[i] = vx_integ[i-1] + ax_200[i] * 0.025

    # vx, vy, w, ax, ay = data[2]
    # vx, vy, w, ax, ay = vx[1], vy[1], w[1], ax[1], ay[1]
    # x, y, theta = [0], [0], [0]
    # for i in range(len(vx)-1):
    #     x_dot = vx[i]*cos(theta[i]) - vy[i]*sin(theta[i])
    #     y_dot = vx[i]*sin(theta[i]) + vy[i]*sin(theta[i])

    #     x.append(x[-1] + 0.025 * x_dot)
    #     y.append(y[-1] + 0.025 * y_dot)
    #     theta.append(theta[-1] + 0.025 * w[i])

    # # map_file = open(f"/home/nick/Desktop/DV-Testing-2/run_{run}/mapLogLog.txt", 'r')
    # # map_data = [np.array(line.split(), float) for line in map_file.readlines()]
    # # add = False
    # # for i in range(1, len(map_data)+1):
    # #     try:
    # #         # print((map_data[-i]))
    # #         if len(map_data[-i])==1 and add:
    # #             break
    # #         if len(map_data[-i])==1 and not add:
    # #             add = True
    # #         if add:
    # #             if map_data[-i][0]==0:
    # #                 plt.scatter(map_data[-i][1]+37, map_data[-i][2], color = 'yellow')
    # #             elif map_data[-i][0]==1:
    # #                 plt.scatter(map_data[-i][1]+37, map_data[-i][2], color = 'blue')
    # #     except:continue

    # plt.scatter(x, y, c = ax, cmap='inferno')
    # plt.title(f'Run {run}')
    # plt.colorbar()
    # plt.grid()
    # plt.show()
    # return

    selected = [[], [0], [], [0]]
    legends = [['actual'],['target', 'target_velocity'], ['vx'], ['actual']]
    if(max(data[3][0][1])==0):
        return
    # print(np.shape(data[4]))
    
    # for i in range(len(data[1][1][1])):
    #     data[1][1][1][i] /= 3.9
    for i in range(len(selected)):
        for k in range(len(selected[i])):
            j = selected[i][k]
            try:
                plt.plot(data[i][j][0], np.array(data[i][j][1])/max(np.abs(data[i][j][1])**0), label =legends[i][k] )
            except Exception as e:
                print(f'{i} {j}: {repr(e)}')

    # plt.plot(data[3][0][0], vx_integ, label='v_integ')
    # plt.plot(data[1][1][0], vc[0]/max(np.abs(vc[0])), label='vx_comp')
    # plt.plot(data[1][1][0], vc[1]/max(np.abs(vc[0])), label='vy_comp')
    # plt.plot(t2, (np.diff(np.array(data[1][1][1]))/25e-3)/max(data[1][1][1]))
    plt.title(f'Run {run}')
    plt.legend()
    plt.grid()
    plt.show()

runs = os.listdir('/home/nick/Desktop/Prom Racing/Testing_Data/DV-complete/DV-Testing-2')
for run in runs:
    if 'run' not in run:continue
    run = int(run[4:6])
    log_from_run(run)