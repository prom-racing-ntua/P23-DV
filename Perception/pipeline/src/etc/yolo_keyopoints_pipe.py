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

from cnn import *

# Test Images
img1 = cv2.imread('/content/drive/MyDrive/photos_lab/IMG_20220929_194223.jpg')
img2 = cv2.imread('/content/drive/MyDrive/photos_lab/IMG_20220929_194225.jpg')
imgs = []
imgs.append(img1)
imgs.append(img2)

plt.figure()
plt.imshow(img1)

plt.figure()
plt.imshow(img2)



def initYOLOModel(modelpath):
  # Load YoloV5 and run inference
  modelpath= '/content/drive/MyDrive/best.pt'
  yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', modelpath)
  results = yolo_model(imgs, size=640)

  return results


# Initialilze Keypoints CNN
def initKeypoint(modelpath):
  model = BasicCNN()
  modelpath = '/content/drive/MyDrive/Prom Racing All/A. Planning/A11. Driverless Software/Driverless 2021-2022/Πέτρος Γεωργούλας Ρέιτ/KeypointsModel.pt'
  model.load_state_dict(torch.load(modelpath,map_location=torch.device('cpu')))

  return model


# Input: yolo results, img from camera, imgs = [img]
# Output: cones_list, classes_list, or_dimensions
def cropResizeCones(yolo_results, images):
  cones_list = []
  classes_list = []
  or_dimensions = []

  # Get Cones from Yolo results
  for i in range(len(images)):
    img_cones = yolo_results.pandas().xyxy[i]
    img_cones_list = []
    img_classes_list = []
    or_dimensions_list = []
    for j in range(len(img_cones)):
      # Convert cones to images
      cone_dict = img_cones.iloc[j].to_dict()
      cone = images[i][math.floor(cone_dict['ymin']):math.floor(cone_dict['ymax']),math.floor(cone_dict['xmin']):math.floor(cone_dict['xmax'])]
      or_dimensions_list.append([cone_dict['xmin'], cone_dict['ymin'], cone_dict['xmax'], cone_dict['ymax']])

      # Resize Cones
      cone = cv2.resize(cone,dsize=(48,65))
      img_cones_list.append(cone)
      img_classes_list.append(cone_dict['class'])
    cones_list.append(img_cones_list)
    classes_list.append(img_classes_list)
    or_dimensions.append(or_dimensions_list)

    return cones_list, classes_list, or_dimensions

# Input: cones_list, keypoints_model
# Output: predictions 
def runKeypoints(cones_list, keypoints_model, images):
  test_images = []
  for i in range(len(images)):
    for j in range(len(cones_list[i])):
      test_images.append(torch.from_numpy(cones_list[i][j]).permute(2,0,1).unsqueeze(0).float())
    
  test_images = torch.cat(test_images, dim=0)

  predictions = keypoints_model(test_images.float())
  predictions = predictions.cpu().detach().numpy()
  predictions = predictions.reshape(predictions.shape[0], 7, 2)
# End Function

def finalCoordinates(cones_list, or_dimensions, predictions):
  curr = 0
  for i in range(len(imgs)):
    for j in range(len(cones_list[i])):
      for k in range(7):
        x = math.floor(predictions[curr+j][k][0]*(or_dimensions[i][j][2]-or_dimensions[i][j][0])/48) + math.floor(or_dimensions[i][j][0])
        y = math.floor(predictions[curr+j][k][1]*(or_dimensions[i][j][3]-or_dimensions[i][j][1])/65) + math.floor(or_dimensions[i][j][1]) 
    curr+=len(cones_list[i])

# solvepnp