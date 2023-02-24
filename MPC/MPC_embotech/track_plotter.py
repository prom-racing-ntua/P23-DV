import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import Trackdrive as td


class ConeColor(Enum):
    YELLOW = 0,
    BLUE = 1,
    ORANGE = 2,
    BIG_ORANGE = 3


def get_circle_point(center, radius, starting_angle, stopping_angle, number_of_cones):
    points_x = []
    points_y = []
    
    for i in range(number_of_cones+1):
        ang = starting_angle + i * (stopping_angle - starting_angle) / number_of_cones
        points_x.append(center[0] + np.cos(ang)*radius)
        points_y.append(center[1] + np.sin(ang)*radius)
    return points_x, points_y


# Plot the course of a vehicle given its velocity
def plot_course(axes, velocities, starting_position=[0,0,0], time_step=0.1):
    X = np.arange(velocities[0].size)
    pos = np.array([starting_position])
    # get change of position and yaw
    delta = velocities * time_step

    # calculate all the positions of the vehicle
    for i in range(X.size):
        dx = delta[0,i]
        dy = delta[1,i]
        # dy = 0
        dtheta = delta[2,i]
        new_head = pos[i, 2] + dtheta
        ct = np.cos(pos[i, 2])
        st = np.sin(pos[i, 2])
        dN = dx*ct - dy*st
        dE = dx*st + dy*ct
        new_pos = np.array([pos[i, 0]+dN, pos[i,1]+dE, new_head])
        pos = np.append(pos, [new_pos], axis=0)

    # plot course
    axes.plot(pos[:,1], pos[:,0])
    return pos[:,1], pos[:,0]


def getSkidpad(axes, return_points=False):
    y_back = np.sqrt((15.25/2 + 3)**2 - (1.5-9.125)**2)
    y_front = - y_back

    outer_left_x = [-1.5, -1.5]
    outer_left_y = [y_back+5, y_back]

    outer_right_x = [1.5, 1.5, 1.5]
    outer_right_y = [y_front-10, y_front-5, y_front]

    inner_left_x, inner_left_y = get_circle_point([-9.125, 0], 15.25/2, 0, 2*np.pi, 16)
    inner_right_x, inner_right_y = get_circle_point([9.125, 0], 15.25/2, 0, 2*np.pi, 16)

    x, y = get_circle_point([-9.125, 0], 15.25/2+3, 3/8*np.pi, 2*np.pi-3/8*np.pi, 10)
    outer_left_x += x
    outer_left_y += y

    x, y = get_circle_point([9.125, 0], 15.25/2+3, -5/8*np.pi, 5/8*np.pi, 10)
    outer_right_x += x
    outer_right_y += y

    outer_left_x.append(-1.5)
    outer_left_x.append(-1.5)
    outer_left_x.append(-1.5)
    outer_left_y.append(y_front)
    outer_left_y.append(y_front-5)
    outer_left_y.append(y_front-10)

    outer_right_x.append(1.5)
    outer_right_x.append(1.5)
    outer_right_y.append(y_back)
    outer_right_y.append(y_back+5)

    axes.plot([-1.5, 1.5], [0,0], color='darkorange', linestyle='--', linewidth='3')
    axes.plot(inner_right_x, inner_right_y, color='yellow', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='orange')
    axes.plot(inner_left_x, inner_left_y, color='lightblue', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='blue')
    axes.plot(outer_right_x, outer_right_y, color='lightblue', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='blue')
    axes.plot(outer_left_x, outer_left_y, color='yellow', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='orange')
    axes.plot([-1.5,1.5], [0,0], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    axes.axis('equal')

    if return_points:
        inner_right_x.extend(outer_left_x)
        inner_right_y.extend(outer_left_y)
        inner_left_x.extend(outer_right_x)
        inner_left_y.extend(outer_right_y)
        yellow_x = np.array(inner_right_x)
        yellow_y = np.array(inner_right_y)
        blue_x = np.array(inner_left_x)
        blue_y = np.array(inner_left_y)
        print(yellow_x)
        print(yellow_y)
        yellow_cones = np.stack((yellow_x, yellow_y, np.array([ConeColor.YELLOW]*yellow_x.size)), axis=1)
        blue_cones = np.stack((blue_x, blue_y, np.array([ConeColor.BLUE]*blue_x.size)), axis=1)
        return np.concatenate((yellow_cones, blue_cones))


def getAccel(axes, return_points=False):
    blue_x = []
    blue_y = []

    yellow_x = []
    yellow_y = []

    y = -2.5
    for i in range(23):
        blue_x.append(-2.5)
        yellow_x.append(2.5)
 
        blue_y.append(y)
        yellow_y.append(y)
        y += 2.5
    
    axes.plot([-2.5,2.5], [0,0], color='darkorange', linestyle='--', linewidth='3')
    axes.plot([-2.5,2.5], [45,45], color='darkorange', linestyle='--', linewidth='3')
    axes.plot(blue_x, blue_y, color='lightblue', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='blue')
    axes.plot(yellow_x, yellow_y, color='yellow', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='orange')
    axes.plot([-2.5,2.5], [0,0], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    axes.plot([-2.5,2.5], [45,45], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    axes.axis('equal')

    if return_points:
        yellow_x = np.array(yellow_x)
        yellow_y = np.array(yellow_y)
        blue_x = np.array(blue_x)
        blue_y = np.array(blue_y)
        yellow_cones = np.stack((yellow_x, yellow_y, np.array([ConeColor.YELLOW]*yellow_x.size)), axis=1)
        blue_cones = np.stack((blue_x, blue_y, np.array([ConeColor.BLUE]*blue_x.size)), axis=1)
        return np.concatenate((yellow_cones, blue_cones))


# Plot of real autoX track
def getAutoX(axes, return_points=False):
    # set all cones in track
    yellow_cones_x = np.array([2, 2, 2, 2, 2, 2, -2, -4.5, -7, -9.5, -12, -14.5, -17, -19.5, -22, -22, -22, -22, -22, -22, -24.5, -27, -29.5, -32, -34.5, -37, -37, -37, -37, -37, -37, -34.5,-32, -29.5, -27, -24.5, -22, -19.5, -17, -14.5, -12, -9.5, -7, -4.5, -2, 2, 2, 2, 2])
    yellow_cones_y = np.array([0, 2.5, 5, 7.5, 10, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 10, 7.5, 5, 2.5, 0, 0, 0, 0, 0, 0, 0, -2.5, -5, -7.5, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -7.5, -5, -2.5])

    blue_cones_x = np.array([-2, -2, -2, -2, -4.5, -7, -9.5, -12, -14.5, -17, -17, -17, -17, -17, -17, -19.5, -22, -24.5, -27, -29.5, -32, -14.5, -12, -9.5, -7, -4.5, -2, -2])
    blue_cones_y = np.array([0, 2.5, 5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 5, 2.5, 0, -2.5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -2.5])

    axes.plot([-2, 2], [0,0], color='darkorange', linestyle='--', linewidth='3')
    axes.plot(blue_cones_x, blue_cones_y, color='lightblue', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='blue')
    axes.plot(yellow_cones_x, yellow_cones_y, color='yellow', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='orange')
    axes.plot([-2,-2,2,2], [0.75,-0.75,0.75,-0.75], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    axes.set_xlim([-45, 10])
    axes.set_ylim([-25, 30])

    if return_points:
        yellow_cones = np.stack((yellow_cones_x, yellow_cones_y, np.array([ConeColor.YELLOW]*yellow_cones_x.size)), axis=1)
        blue_cones = np.stack((blue_cones_x, blue_cones_y, np.array([ConeColor.BLUE]*blue_cones_x.size)), axis=1)
        return np.concatenate((yellow_cones, blue_cones))


def getTrackdrive(axes, return_points=False):
    cones_blue = np.flip(td.cones_blue[:,:2], 1)
    cones_blue = np.concatenate((cones_blue, np.array([[ConeColor.BLUE]*cones_blue[:,0].size]).T), axis=1)

    cones_yellow = np.flip(td.cones_yellow[:,:2], 1)
    cones_yellow = np.concatenate((cones_yellow, np.array([[ConeColor.YELLOW]*cones_yellow[:,0].size]).T), axis=1)

    cones_orange_big = np.flip(td.cones_orange_big[:,:2], 1)
    cones_orange_big = np.concatenate((cones_orange_big, np.array([[ConeColor.BIG_ORANGE]*cones_orange_big[:,0].size]).T), axis=1)

    axes.plot(cones_blue[:,0], cones_blue[:,1], color='lightblue', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='blue')
    axes.plot(cones_yellow[:,0], cones_yellow[:,1], color='yellow', linestyle='-.', linewidth='1.5', marker='o', markerfacecolor='orange')
    axes.plot(cones_orange_big[:,0], cones_orange_big[:,1], color='None', marker='o', markerfacecolor='darkorange', markeredgecolor='None', markersize=7)
    axes.axis('equal')

    if return_points:
        return np.concatenate((cones_yellow, cones_blue))


if __name__=='__main__':
    fig, ax = plt.subplots()
    fig.set_size_inches(9,9)
    ax.set_title("Vehicle Course")
    getAutoX(ax)

    plt.show()