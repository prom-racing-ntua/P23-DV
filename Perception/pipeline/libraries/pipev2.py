import cv2
import torch
from PIL import Image

import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import math

from torch.utils.data import Dataset
import json
import torchvision
from torch.utils.data import SubsetRandomSampler, DataLoader

import torch.optim as optim

from .cnnv2 import *


def initYOLOModel(modelpath, conf=0.25, iou=0.45):
    yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', modelpath)
    yolo_model.agnostic = True
    yolo_model.conf = conf
    yolo_model.iou = iou
    return yolo_model


def inferenceYOLO(model, imgpath, res):
    results = model(imgpath, size=res)
    return results


def initKeypoint(modelpath):
    model = BasicCNN()
    model.load_state_dict(torch.load(modelpath,map_location=torch.device('cpu')))
    return model


def cropResizeCones(yolo_results, image, margin):
    img_h = len(image)
    img_w = len(image[0])
    bounding_boxes = yolo_results.pandas().xyxy[0]
    cones_imgs = []
    classes = []
    cropped_img_corners = []
    for j in range(len(bounding_boxes)):
        bb_dict = bounding_boxes.iloc[j].to_dict()
        
        # Find corners of cropped images
        xmin = max(math.floor(bb_dict['xmin'])-margin, 0)
        xmax = min(math.floor(bb_dict['xmax'])+margin, img_w-1)
        ymin = max(math.floor(bb_dict['ymin'])-margin, 0)
        ymax = min(math.floor(bb_dict['ymax'])+margin, img_h-1)
        
        # Stack cropped images
        cone_img = image[ymin:ymax, xmin:xmax]
        cone_img = cv2.resize(cone_img,dsize=(48,64))
        cones_imgs.append(cone_img)
        
        # Stack corners of cropped images
        cropped_img_corners.append([xmin, ymin, xmax, ymax])

        # Stack classes of cropped images
        classes.append(bb_dict['class'])

    return cones_imgs, classes, cropped_img_corners


def runKeypoints(cones_imgs, keypoints_model):
    cones_imgs_list = []
    
    # Convert images to a Pytorch-friendly format
    for j in range(len(cones_imgs)):
        cones_imgs_list.append(torch.from_numpy(cones_imgs[j].transpose(2,0,1)).unsqueeze(0).float())
    cones_imgs_tensor = torch.cat(cones_imgs_list, dim=0)
    
    # Inference
    predictions = keypoints_model(cones_imgs_tensor).cpu().detach().numpy()
    predictions = predictions.reshape(predictions.shape[0], 7, 2)

    return predictions


def rt_converter(camera, pnp_dist):
    # Takes distance calculated by solvePnP and calculates range,theta from car CoG based on the camera used
    if camera == 'left':
        x = np.cos(math.pi*34/180)*np.cos(math.pi*9.5/180)*pnp_dist[2] + np.sin(math.pi*34/180)*pnp_dist[0] - np.sin(math.pi*9.5/180)*np.cos(math.pi*34/180)*pnp_dist[1] - 31
        y = -np.sin(math.pi*34/180)*np.cos(math.pi*9.5/180)*pnp_dist[2] + np.cos(math.pi*34/180)*pnp_dist[0] + np.sin(math.pi*34/180)*np.sin(math.pi*9.5/180)*pnp_dist[1] - 10
        r = np.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
    elif camera == 'right':
        x = np.cos(math.pi*34/180)*np.cos(math.pi*9.5/180)*pnp_dist[2] - np.sin(math.pi*34/180)*pnp_dist[0] - np.sin(math.pi*9.5/180)*np.cos(math.pi*34/180)*pnp_dist[1] - 31
        y = np.sin(math.pi*34/180)*np.cos(math.pi*9.5/180)*pnp_dist[2] + np.cos(math.pi*34/180)*pnp_dist[0] - np.sin(math.pi*34/180)*np.sin(math.pi*9.5/180)*pnp_dist[1] + 10
        r = np.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
    elif camera == 'center':
        x = np.cos(math.pi*9.5/180)*pnp_dist[2] - np.sin(math.pi*9.5/180)*pnp_dist[1] - 30
        y = pnp_dist[0]
        r = np.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
    return round(r[0]/100,3), round(theta,4)


def finalCoordinates(camera, classes, cropped_img_corners, predictions, OffsetY):
    rt = []
    for j in range(len(classes)):
        cone_keypoints = []
        
        # Outputs the coordinates of keypoints on the full image (not just ROI)
        for k in range(7):
            x = predictions[j][k][0]*(cropped_img_corners[j][2] - cropped_img_corners[j][0])/48 + cropped_img_corners[j][0]
            y = OffsetY + predictions[j][k][1]*(cropped_img_corners[j][3] - cropped_img_corners[j][1])/64 + cropped_img_corners[j][1]
            cone_keypoints.append([x,y]) 
        cone_keypoints_numpy = np.array(cone_keypoints)
        
        # Depending on lens used choose an intrinsic camera matrix
        if camera == 'center':
            cameraMatrix = np.array([[2500, 0, 640], [0, 2500, 512], [0, 0, 1]])
            distCoeffs = np.array([[0, 0, 0, 0, 0]])
        elif (camera=='left' or camera=='right'):
            cameraMatrix = np.array([[1250, 0, 640], [0, 1250, 512], [0, 0, 1]])
            distCoeffs = np.array([[0, 0, 0, 0, 0]])            
        
        # Depending on the cone class set real coordiantes of keypoints
        if classes[j] == 3:
            real_coords = np.array([[7.4, 2.7, 0],
                        [5.8, 11.9, 0],
                        [4.3, 20.5, 0],
                        [0, 32.5 ,0],
                        [-4.3, 20.5, 0],
                        [-5.8, 11.9, 0],
                        [-7.4, 2.7, 0]]) #WROOOONG
        elif classes[j] < 3:
            real_coords = np.array([[7.4, 2.7, 0],
                        [5.8, 11.9, 0],
                        [4.3, 20.5, 0],
                        [0, 32.5 ,0],
                        [-4.3, 20.5, 0],
                        [-5.8, 11.9, 0],
                        [-7.4, 2.7, 0]])
        
        # Use solvePnP to get cone position in camera frame and then find range,theta from car CoG
        _, _, translation_vector = cv2.solvePnP(real_coords, cone_keypoints_numpy, cameraMatrix, distCoeffs, cv2.SOLVEPNP_IPPE)
        print(translation_vector)
        rt.append(rt_converter(camera, translation_vector))

    return rt

def runPerception(img_path, camera, yolo_path, keypoints_path, OffsetY):
    
    # Initialize YOLO and Keypoints model
    yolo_model = initYOLOModel(yolo_path)
    keypoints_model = initKeypoint(keypoints_path)
    
    # Read image and convert to RGB
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Get YOLO results
    yolo_results = inferenceYOLO(yolo_model, img, 1280)

    # Get bounded images
    cones_imgs, classes, cropped_img_corners = cropResizeCones(yolo_results, img, 3)
    
    if len(cones_imgs) > 0:
        # Get Keypoints predictions
        keypoints_predictions = runKeypoints(cones_imgs, keypoints_model)
        # Get range,theta for each cone in car frame
        final_coordinates = finalCoordinates(camera, classes, cropped_img_corners, keypoints_predictions, OffsetY)
        for i in range(len(classes)):
            print("Class: ", classes[i], " Range: ", final_coordinates[i][0], " Theta: ", final_coordinates[i][1])
        return classes, final_coordinates
    else:
        print("No cones found")
        return