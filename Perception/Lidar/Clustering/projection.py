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
    pitch = np.abs(8.0) #to be measured
    lidar_cog_dx = np.abs(110.6) #in cms 
    lidar_cog_dz = np.abs(26.6)
    lidar_points = np.array([[-lidar_point[1]], [-lidar_point[0]], [-lidar_point[2]]])
    cog2lidar_rot = pitch_matrix(-pitch)
    cog2lidar_tr = np.array([[lidar_cog_dx], [0], [-lidar_cog_dz]])
    # print("check pipeline 1")
    # print(lidar_points)
    # print(cog2lidar_rot @ lidar_points)
    # print(cog2lidar_tr)
    checkpoint1 = (cog2lidar_rot @ lidar_points) + cog2lidar_tr
    # print(checkpoint1)
    
    #second part (cam -> cog  -> cone)
    if (camera_str == "center"): 
        pitch2 = np.abs(9.0) 
        yaw2=np.abs(0.0)
        cog2cam_rot = pitch_matrix(-pitch2)
        cog2cam_tr = np.array([[-30], [0], [-100]])
    if (camera_str == "left"):
        pitch2 = np.abs(9.0) #to be measured
        yaw2=np.abs(35.0)
        cog2cam_rot = pitch_matrix(-pitch2)@yaw_matrix(-yaw2)
        cog2cam_tr = np.array([[-31], [-10], [-105]])
    if (camera_str == "right"):
        pitch2 = np.abs(9.0) #to be measured
        yaw2=np.abs(35)
        cog2cam_rot = pitch_matrix(-pitch2) @yaw_matrix(yaw2)
        cog2cam_tr = np.array([[-31], [10], [-105]])
    checkpoint2 = -(cog2cam_rot.T@cog2cam_tr) + (cog2cam_rot.T@checkpoint1)
    checkpoint_out = np.array([checkpoint2[1],checkpoint2[2],checkpoint2[0]])
    return checkpoint_out/100, checkpoint1/100


def cam_2_pixel(camera_str,cam_point):
    if (camera_str == "center"):
        fru = 2500
        frv = fru
        cx = 640
        cy = 512
    if (camera_str == "left" or camera_str == "right"):
        fru = 1250
        frv = fru
        cx = 640
        cy = 512
    # print(cam_point[0][0], " ", cam_point[1][0], " ",cam_point[2][0])
    u = int(fru*(cam_point[0][0]/cam_point[2][0]) + cx)
    v = int(frv*(cam_point[1][0]/cam_point[2][0]) + cy)
    return np.array([u,v])

clusters=pd.read_csv("DataError/output_clustering.csv").to_numpy()
print(np.shape(clusters),type(clusters))
for i in range(3):
    if(i==0):
        camera_selection="center"
        image=cv2.imread("DataKsi2/camera/test_center.png")
        print(np.shape(image))
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]!="--"):
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                res3 = cam_2_pixel(camera_selection,100*res2)
                u=res3[0]
                v=res3[1]
                image[v-4:v+4, u-4:u+4] = np.array([0,0,255])
        cv2.imshow("projection center",image)
    if(i==1):
        camera_selection="left"
        image=cv2.imread("DataKsi2/camera/test_left.png")
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]!="--"):
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                res3 = cam_2_pixel(camera_selection,100*res2)
                u=res3[0]
                v=res3[1]
                image[v-4:v+4, u-4:u+4] = np.array([0,0,255])
        cv2.imshow("projection left",image)
    if(i==2):
        camera_selection="right"
        image=cv2.imread("DataKsi2/camera/test_right.png")
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]!="--"):
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                print("lidar center point ",j)
                print("from lidar points",input_point,"and distance ",np.sqrt(input_point[0]**2+input_point[1]**2))
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                print("from cog points",np.squeeze(res1),"and distance ",np.sqrt(np.squeeze(res1)[0]**2+np.squeeze(res1)[1]**2))
                print("from camera points",np.squeeze(res2),"and distance ",np.sqrt(np.squeeze(res2)[0]**2+np.squeeze(res2)[2]**2))
                res3 = cam_2_pixel(camera_selection,res2)
                u=res3[0]
                v=res3[1]
                image[v-4:v+4, u-4:u+4] = np.array([0,0,255])
        cv2.imshow("projection right",image)
    print("")
cv2.waitKey(0)
cv2.destroyAllWindows()     



    

    
