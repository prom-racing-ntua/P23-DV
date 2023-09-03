import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2

cameraMatrix = np.array([[2500.0, 0.0, 640.0], [0, 2500.0, 512.0], [0.0, 0.0, 1.0]]).astype(float)
cameraMatrix2 = np.array([[1250.0, 0, 640.0], [0, 1250.0, 512.0], [0.0, 0.0, 1.0]]).astype(float)
distCoeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).astype(float)
width = 100
height = 300
flag = 0
prev_flag = 0
scale_up = 2.0

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
    
# def lidar_2_camera(camera_str, lidar_point): #logiki cog -> lidar -> cone
#                                              #       cam -> cog  -> cone 
#     #first part (cog -> lidar -> cone)
#     pitch = np.abs(8.0) #to be measured
#     lidar_cog_dx = np.abs(110.6) #in cms 
#     lidar_cog_dz = np.abs(26.6)
#     lidar_points = np.array([[-lidar_point[1]], [-lidar_point[0]], [-lidar_point[2]]])
#     cog2lidar_rot = pitch_matrix(-pitch)
#     cog2lidar_tr = np.array([[lidar_cog_dx], [0], [-lidar_cog_dz]])
#     # print("check pipeline 1")
#     # print(lidar_points)
#     # print(cog2lidar_rot @ lidar_points)
#     # print(cog2lidar_tr)
#     checkpoint1 = (cog2lidar_rot @ lidar_points) + cog2lidar_tr
#     # print(checkpoint1)
    
#     #second part (cam -> cog  -> cone)
#     if (camera_str == "center"): 
#         pitch2 = np.abs(12.0) 
#         yaw2=np.abs(0.0)
#         cog2cam_rot = pitch_matrix(-pitch2)
#         cog2cam_tr = np.array([[-30], [0], [-100]])
#     if (camera_str == "left"):
#         pitch2 = np.abs(12.0) #to be measured
#         yaw2=np.abs(35.0)
#         cog2cam_rot = pitch_matrix(-pitch2)@yaw_matrix(-yaw2)
#         cog2cam_tr = np.array([[-31], [-10], [-105]])
#     if (camera_str == "right"):
#         pitch2 = np.abs(12.0) #to be measured
#         yaw2=np.abs(35)
#         cog2cam_rot = pitch_matrix(-pitch2) @yaw_matrix(yaw2)
#         cog2cam_tr = np.array([[-31], [10], [-105]])
#     checkpoint2 = -(cog2cam_rot.T@cog2cam_tr) + (cog2cam_rot.T@checkpoint1)
#     checkpoint_out = np.array([checkpoint2[1],checkpoint2[2],checkpoint2[0]])
#     return checkpoint_out/100, checkpoint1/100

def lidar_2_camera(camera_str, lidar_point):
    pitch = np.abs(12.0) # to be measured
    yaw = np.abs(37.0) 
    lidar_cog_dx = np.abs(80.0) #in cms 
    lidar_cog_dz = np.abs(20.0)
    lidar_points = np.array([[-lidar_point[1]], [-lidar_point[0]], [-lidar_point[2]]])
    cog2lidar_rot = pitch_matrix(-pitch)
    cog2lidar_tr = np.array([[lidar_cog_dx], [0], [-lidar_cog_dz]])
    # print("check pipeline 1")
    # print(lidar_points)
    # print(cog2lidar_rot @ lidar_points)
    # print(cog2lidar_tr)
    checkpoint1 = (cog2lidar_rot @ lidar_points) + cog2lidar_tr
    #new pipeline for camera projection
    dpitch = np.abs(4.0)
    c2l_pitch = pitch_matrix(+dpitch)
    c2g_pitch = pitch_matrix(+pitch)
    c_yaw_pos = yaw_matrix(yaw)
    c_yaw_neg = yaw_matrix(-yaw) 
    c2l_dy_glob = 0.0
    lidar_points = np.array([[-lidar_point[1]], [-lidar_point[0]], [-lidar_point[2]]])
    if(camera_str=="center"):
        c2l_dx_glob = 120.0
        c2l_dz_glob = 45.0  
        c2l_dy_glob = 0.0
        c2l_tr = np.array([[c2l_dx_glob],[c2l_dy_glob],[c2l_dz_glob]])
        checkpoint2 = c2l_pitch@lidar_points+c2g_pitch@c2l_tr
    if(camera_str=="right"):
        c2l_dx_glob = 125.0
        c2l_dz_glob = 50.0 
        c2l_dy_glob = 7.0
        c2l_tr = np.array([[c2l_dx_glob],[c2l_dy_glob],[c2l_dz_glob]]) 
        checkpoint2 = c2l_pitch@c_yaw_neg@lidar_points+c2g_pitch@c_yaw_neg@c2l_tr
    if(camera_str=="left"):
        c2l_dx_glob = 125.0
        c2l_dz_glob = 50.0 
        c2l_dy_glob = -5.0
        c2l_tr = np.array([[c2l_dx_glob],[c2l_dy_glob],[c2l_dz_glob]])  
        checkpoint2 = c2l_pitch@c_yaw_pos@lidar_points+c2g_pitch@c_yaw_pos@c2l_tr
    checkpoint_out = np.array([checkpoint2[1],checkpoint2[2],checkpoint2[0]])
    return checkpoint_out/100, checkpoint1/100   

def normalize_xs(x_bef):
    if(x_bef<0): return 1
    if(x_bef>1280): return 1280-2
    else: return x_bef
    
def normalize_ys(y_bef):
    if(y_bef<0): return 1
    if(y_bef>1280): return 1280-2
    else: return y_bef

def cam_2_pixel(cam_str,cam_point):
    c2i_tr = np.array([[0.0,0.0,0.0]])
    c2i_rot = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    if(cam_str=="center"): proj_ = cv2.projectPoints(cam_point,c2i_rot,c2i_tr,cameraMatrix,distCoeffs)[0].reshape(1,2).squeeze()
    if(cam_str=="left" or cam_str=="right"): proj_ = cv2.projectPoints(cam_point,c2i_rot,c2i_tr,cameraMatrix2,distCoeffs)[0].reshape(1,2).squeeze()
    u = int(proj_[0])
    v = int(proj_[1])
    return np.array([u,v]) 
    
clusters=pd.read_csv("DataKsi2/distances/output_clustering.csv").to_numpy()
print(np.shape(clusters),type(clusters))
for i in range(3):
    if(i==0):
        u_array = []
        v_array = []
        camera_selection="center"
        image=cv2.imread("DataKsi2/camera/test_center.png")
        print(np.shape(image))
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]=="--"): flag = 1
            if(clusters[j][0]!="--"):
                flag = 0
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                # input_point = np.array([0.608,-1.726,-0.096])
                print("camera_selection -> ",camera_selection)
                print("lidar center point ",j)
                print("from lidar points",input_point,"and distance ",np.sqrt(input_point[0]**2+input_point[1]**2))
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                print("from cog points",np.squeeze(res1),"and distance ",np.sqrt(np.squeeze(res1)[0]**2+np.squeeze(res1)[1]**2))
                print("from camera points",np.squeeze(res2),"and distance ",np.sqrt(np.squeeze(res2)[0]**2+np.squeeze(res2)[2]**2))
                res3 = cam_2_pixel("center",100*res2)
                u=res3[0]
                v=res3[1]
                if(u>=0 and u<1280 and v>=0 and v<1024): 
                    u_array.append(u)
                    v_array.append(v)
                    # image = cv2.rectangle(image, end_point,start_point, (255,0,0), 2)
                    image[v-4:v+4, u-4:u+4] = np.array([0,0,255])
            if(j==np.shape(clusters)[0]-1): flag = 1
            if(np.shape(u_array)[0]>0 and np.shape(v_array)[0]>0 and flag==1):
                print("u and v arrays for center bounding boxes are:")
                print(u_array)
                print(v_array)
                u_array = np.array(u_array)
                v_array = np.array(v_array)
                x1 = int(np.min(u_array))
                y1 = int(np.min(v_array))
                x2 = int(np.max(u_array))
                y2 = int(np.max(v_array))
                width_new = scale_up*np.abs(x2-x1)
                height_new = width_new
                x1_new = normalize_xs(int(x1-(width_new/2)))
                x2_new = normalize_xs(int(x2+(width_new/2)))
                y1_new = normalize_ys(int(y1-(height_new/2)))
                y2_new = normalize_ys(int(y2+(height_new/2)))
                start_point = (x1_new,y1_new)
                end_point = (x2_new,y2_new)
                image = cv2.rectangle(image, start_point, end_point, (255,0,0), 2)
                print("on centre i have xs_final: ",x1_new, " ", x2_new)
                print("on centre i have ys_final: ",y1_new, " ", y2_new)
                u_array = []
                v_array = []
        cv2.imshow("projection center",image)
    if(i==1):
        u_array = []
        v_array = []
        camera_selection="left" #++
        image_2=cv2.imread("DataKsi2/camera/test_left.png") #++
        print(np.shape(image))
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]=="--"): flag = 1
            if(clusters[j][0]!="--"):
                flag = 0
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                # input_point = np.array([0.608,-1.726,-0.096])
                print("camera_selection -> ",camera_selection)
                print("lidar center point ",j)
                print("from lidar points",input_point,"and distance ",np.sqrt(input_point[0]**2+input_point[1]**2))
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                print("from cog points",np.squeeze(res1),"and distance ",np.sqrt(np.squeeze(res1)[0]**2+np.squeeze(res1)[1]**2))
                print("from camera points",np.squeeze(res2),"and distance ",np.sqrt(np.squeeze(res2)[0]**2+np.squeeze(res2)[2]**2))
                res3 = cam_2_pixel("left",100*res2)
                u=res3[0]
                v=res3[1]
                if(u>=0 and u<1280 and v>=0 and v<1024): 
                    u_array.append(u)
                    v_array.append(v)
                    # image = cv2.rectangle(image, end_point,start_point, (255,0,0), 2)
                    image_2[v-4:v+4, u-4:u+4] = np.array([0,0,255])
            if(j==np.shape(clusters)[0]-1): flag = 1
            if(np.shape(u_array)[0]>0 and np.shape(v_array)[0]>0 and flag==1):
                print(u_array)
                print(v_array)
                u_array = np.array(u_array)
                v_array = np.array(v_array)
                x1 = int(np.min(u_array))
                y1 = int(np.min(v_array))
                x2 = int(np.max(u_array))
                y2 = int(np.max(v_array))
                width_new = scale_up*np.abs(x2-x1)
                height_new = width_new
                x1_new = normalize_xs(int(x1-(width_new/2)))
                x2_new = normalize_xs(int(x2+(width_new/2)))
                y1_new = normalize_ys(int(y1-(height_new/2)))
                y2_new = normalize_ys(int(y2+(height_new/2)))
                start_point = (x1_new,y1_new)
                end_point = (x2_new,y2_new)
                image_2 = cv2.rectangle(image_2, start_point, end_point, (255,0,0), 2)
                print("on lefty i have xs_final: ",x1_new, " ", x2_new)
                print("on lefty i have ys_final: ",y1_new, " ", y2_new)
                u_array = []
                v_array = []
        cv2.imshow("projection left",image_2) #++
    if(i==2):
        u_array = []
        v_array = []
        camera_selection="right"
        image_3=cv2.imread("DataKsi2/camera/test_right.png")
        print(np.shape(image))
        for j in range (np.shape(clusters)[0]):
            if(clusters[j][0]=="--"): flag = 1
            if(clusters[j][0]!="--"):
                flag = 0
                input_point=np.array([float(clusters[j][0]),float(clusters[j][1]),float(clusters[j][2])])
                # input_point = np.array([0.608,-1.726,-0.096])
                print("camera_selection -> ",camera_selection)
                print("lidar center point ",j)
                print("from lidar points",input_point,"and distance ",np.sqrt(input_point[0]**2+input_point[1]**2))
                res2,res1 = lidar_2_camera(camera_selection,100*input_point)
                print("from cog points",np.squeeze(res1),"and distance ",np.sqrt(np.squeeze(res1)[0]**2+np.squeeze(res1)[1]**2))
                print("from camera points",np.squeeze(res2),"and distance ",np.sqrt(np.squeeze(res2)[0]**2+np.squeeze(res2)[2]**2))
                res3 = cam_2_pixel("right",100*res2)
                u=res3[0]
                v=res3[1]
                if(u>=0 and u<1280 and v>=0 and v<1024): 
                    u_array.append(u)
                    v_array.append(v)
                    # image = cv2.rectangle(image, end_point,start_point, (255,0,0), 2)
                    image_3[v-4:v+4, u-4:u+4] = np.array([0,0,255])
            if(j==np.shape(clusters)[0]-1): flag = 1
            if(np.shape(u_array)[0]>0 and np.shape(v_array)[0]>0 and flag==1):
                print("u and v arrays for righty bounding boxes are:")
                print(u_array)
                print(v_array)
                u_array = np.array(u_array)
                v_array = np.array(v_array)
                x1 = int(np.min(u_array))
                y1 = int(np.min(v_array))
                x2 = int(np.max(u_array))
                y2 = int(np.max(v_array))
                width_new = scale_up*np.abs(x2-x1)
                height_new = width_new
                x1_new = normalize_xs(int(x1-(width_new/2)))
                x2_new = normalize_xs(int(x2+(width_new/2)))
                y1_new = normalize_ys(int(y1-(height_new/2)))
                y2_new = normalize_ys(int(y2+(height_new/2)))
                print("on righty i have xs_final: ",x1_new, " ", x2_new)
                print("on righty i have ys_final: ",y1_new, " ", y2_new)
                start_point = (x1_new,y1_new)
                end_point = (x2_new,y2_new)
                image_3 = cv2.rectangle(image_3, start_point, end_point, (255,0,0), 2)
                u_array = []
                v_array = []
        cv2.imshow("projection right",image_3)
    print("")
cv2.waitKey(0)
cv2.destroyAllWindows()     



    

    
