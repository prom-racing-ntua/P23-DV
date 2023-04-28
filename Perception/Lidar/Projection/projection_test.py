import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2

def yaw_matrix(yaw):
    return np.array([[np.cos(math.pi*yaw/180), -np.sin(math.pi*yaw/180), 0],
                    [np.sin(math.pi*yaw/180), np.cos(math.pi*yaw/180), 0],
                    [0, 0, 1]])

def pitch_matrix(pitch):
    return np.array([[np.cos(math.pi*pitch/180), 0, np.sin(math.pi*pitch/180)],
                    [0, 1, 0],
                    [-np.sin(math.pi*pitch/180), 0, np.cos(math.pi*pitch/180)]])

def roll_matrix(roll):
    return np.array([[1, 0, 0],
                    [0,np.cos(math.pi*roll/180), -np.sin(math.pi*roll/180)],
                    [0, np.sin(math.pi*roll/180), np.cos(math.pi*roll/180)]])
    
def lidar_2_camera(camera_str, lidar_point): #logiki cog -> lidar -> cone
                                             #       cam -> cog  -> cone 
    #first part (cog -> lidar -> cone)
    pitch = np.abs(4.0) #to be measured
    lidar_cog_dx = np.abs(50) #in cms 
    lidar_cog_dz = np.abs(30) #to be measured
    lidar_points = np.array([[-lidar_point[1]], [-lidar_point[0]], [-lidar_point[2]]])
    cog2lidar_rot = pitch_matrix(-pitch)
    cog2lidar_tr = np.array([[lidar_cog_dx], [0], [-lidar_cog_dz]])
    checkpoint1 = (cog2lidar_rot @ lidar_points) + cog2lidar_tr
    
    #second part (cam -> cog  -> cone)
    if (camera_str == "center"): 
        pitch2 = np.abs(9.0) 
        yaw2=np.abs(0.0)
        cog2cam_rot = pitch_matrix(-pitch2)@yaw_matrix(yaw2)
        cog2cam_tr = np.array([[-30], [0], [-100]])
        cog2cam_rot_T = np.transpose(cog2cam_rot.T)
        checkpoint2 = -(cog2cam_rot_T@cog2cam_tr) + (cog2cam_rot_T@checkpoint1)
        print(np.shape(checkpoint2))
    if (camera_str == "left"):
        pitch2 = np.abs(9.0) #to be measured
        yaw2=np.abs(35.0)
        cog2cam_rot = pitch_matrix(-pitch2)@yaw_matrix(-yaw2)
        cog2cam_tr = np.array([[-31], [0], [-100]])
        cog2cam_rot_T = np.transpose(cog2cam_rot.T)
        checkpoint2 = -(cog2cam_rot_T@cog2cam_tr) + (cog2cam_rot_T@checkpoint1)
    if (camera_str == "right"):
        pitch2 = np.abs(9.0) #to be measured
        yaw2=np.abs(35)
        rot_temp2 = pitch_matrix(pitch2) @yaw_matrix(yaw2)
        trans_temp2 = np.array([[-31], [10], [-105]])
        checkpoint2 = (rot_temp2 @ checkpoint1) + trans_temp2
    checkpoint_out = np.array([checkpoint2[1],checkpoint2[2],checkpoint2[0]])
    return checkpoint_out/100, checkpoint1/100


def cam_2_pixel(camera_str,cam_point):
    if (camera_str == "center"):
        fru = 2550
        frv = fru
        cx = 640
        cy = 512
    if (camera_str == "left" or camera_str == "right"):
        fru = 1250
        frv = fru
        cx = 640
        cy = 512
    print(cam_point[0][0], " ", cam_point[1][0], " ",cam_point[2][0])
    u = int(fru*(cam_point[0][0]/cam_point[2][0]) + cy)
    v = int(frv*(cam_point[1][0]/cam_point[2][0]) + cx)
    return np.array([u,v])
    
# -1.01394 -6.02681 0.503125
# 1.37077 -2.87256 0.0418077
# 1.37077 -2.87256 0.0418077
input_point=1*np.array([0.0,0.0,0.0]) #wanted in cms 
print(input_point, np.shape(input_point))
res2,res1 = lidar_2_camera("center",100*input_point)
res3 = cam_2_pixel("center",100*res2)
# u=res3[0]
# v=res3[1]
u=50
v=50
print("cog2cone results is: ",res1,np.shape(res1))
print("cam2cone results is: ",res2,np.shape(res2))
print("(u,v) results is: ",res3,np.shape(res3))
input_image=cv2.imread('ultradummy.png')
input_image[v-4:v+4, u-4:u+4] = np.array([0,0,255]) #BGR
cv2.imshow("dummy image",input_image)
cv2.waitKey(0)
# closing all open windows
cv2.destroyAllWindows()

    

    
