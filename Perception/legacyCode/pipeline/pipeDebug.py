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

from .cnn import *

# Input: yolo results, img from camera, imgs = [img]
# Output: cones_list, classes_list, or_dimensions
def cropResizeConesDebug(yolo_results, images, fileName, runFolder, originalImage):
  outPath = fileName.split('.')

  cones_list = []
  classes_list = []
  or_dimensions = []

  # Get Cones from Yolo results
  for i in range(len(images)):
    img_cones = yolo_results.pandas().xyxy[i]
    img_cones_list = []
    img_classes_list = []
    or_dimensions_list = []
    originalImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2RGB)

    for j in range(len(img_cones)):
      # Convert cones to images
      cone_dict = img_cones.iloc[j].to_dict()
      cone = images[i][math.floor(cone_dict['ymin']):math.floor(cone_dict['ymax']),math.floor(cone_dict['xmin']):math.floor(cone_dict['xmax'])]
      or_dimensions_list.append([cone_dict['xmin'], cone_dict['ymin'], cone_dict['xmax'], cone_dict['ymax']])

      # topleft
      startValue = (int(cone_dict['xmin']), int(cone_dict['ymax']))
      # bottomRight
      endValue = (int(cone_dict['xmax']), int(cone_dict['ymin']))

      originalImage = cv2.rectangle(originalImage ,startValue,endValue,(0,255,0),3)

      # Resize Cones
      cone = cv2.resize(cone,dsize=(48,64))
      # DEBUG: Save Cones
      cv2.imwrite(f"{runFolder}/predictions/{outPath[0]}_cones/{outPath[0]}_cone{j}.jpg", cone[...,::-1])

      img_cones_list.append(cone)
      img_classes_list.append(cone_dict['class'])
    cv2.imwrite(f"{runFolder}/predictions/{outPath[0] + '.' + outPath[1]}_bbox.jpg", originalImage)
    cones_list.append(img_cones_list)
    classes_list.append(img_classes_list)
    or_dimensions.append(or_dimensions_list)

    return cones_list, classes_list, or_dimensions

# Input: cones_list, keypoints_model
# Output: predictions 
def runKeypointsDebug(cones_list, keypoints_model, images, fileName, runFolder):
  outPath = fileName.split('.')
  test_images = []
  for i in range(len(images)):
    for j in range(len(cones_list[i])):
        test_images.append(torch.from_numpy((cones_list[i][j].transpose(2,0,1)/255.0)).unsqueeze(0).float())    

  test_images = torch.cat(test_images, dim=0)

  predictions = keypoints_model(test_images.float())
  predictions = predictions.cpu().detach().numpy()
  predictions = predictions.reshape(predictions.shape[0], 7, 2)

  for i in range(len(cones_list[0])):
    for k in range(0,7):
        cones_list[0][i][int(predictions[i][k][1])-1:int(predictions[i][k][1])+1,int(predictions[i][k][0])-1:int(predictions[i][k][0])+1] = np.array([0,255,0])
        # cv2.imwrite(f"{runFolder}/predictions/{outPath[0]}_keypoints{i}.jpg", cones_list[0][i][...,::-1])

  return predictions

def finalCoordinatesDebug(cones_list, or_dimensions, predictions, imgs, cameraMatrix, distCoeffs, objp_orange, originalImage, fileName, runFolder):
  outPath = fileName.split('.')
  curr = 0
  for i in range(len(imgs)):
    pnp_image = []
    for j in range(len(cones_list[i])):
      keypoints_cone = []
      for k in range(7):
        x = predictions[curr+j][k][0]*(or_dimensions[i][j][2] - or_dimensions[i][j][0])/48 + or_dimensions[i][j][0]
        y = predictions[curr+j][k][1]*(or_dimensions[i][j][3] - or_dimensions[i][j][1])/64 + or_dimensions[i][j][1]
        
        # Mark Original Image with keypoints on original dimensions
        originalImage[int(y)-4:int(y)+4,int(x)-4:int(x)+4] = np.array([0,255,0])
        
        keypoints_cone.append([x,y]) 
      # Save Marked Image
      cv2.imwrite(f"{runFolder}/predictions/{outPath[0]+ '.' + outPath[1]}_predictions.jpg", originalImage[...,::-1])
      keypoints_cone_numpy = np.array(keypoints_cone)
      success, rotation_vector, translation_vector = cv2.solvePnP(objp_orange, keypoints_cone_numpy, cameraMatrix, distCoeffs, cv2.SOLVEPNP_IPPE)
      pnp_image.append([success, rotation_vector, translation_vector])

    return pnp_image