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

def initYOLOModel(modelpath, conf=0.25, iou=0.45):

  yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', modelpath)
  yolo_model.agnostic = True
  yolo_model.conf = conf
  yolo_model.iou = iou
  return yolo_model

def inferenceYOLO(model, imgpath, res):
    
    results = model(imgpath, size=res)
    return results


# Initialilze Keypoints CNN
def initKeypoint(modelpath):

  model = ComplexCNN()
  model.load_state_dict(torch.load(modelpath,map_location=torch.device('cpu')))

  return model


# Input: yolo results, img from camera, imgs = [img]
# Output: cones_list, classes_list, or_dimensions
def cropResizeCones(yolo_results, images, originalImage):
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
      cone = cv2.resize(cone,dsize=(48,64))
      # saveImage(Image.fromarray(cone, 'RGB'), f"cone{j}")
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

  for k in range(0,7):
    cones_list[0][0][int(predictions[0][k][1])-1:int(predictions[0][k][1])+1,int(predictions[0][k][0])-1:int(predictions[0][k][0])+1] = np.array([0,255,0])

  # saveImage(Image.fromarray(cones_list[0][0], 'RGB'), "pred")
  return predictions
# End Function

def finalCoordinates(cones_list, or_dimensions, predictions, imgs, cameraMatrix, distCoeffs, objp_orange, monoimg):
  curr = 0
  for i in range(len(imgs)):
    keypoints_image = []
    pnp_image = []
    #print(cones_list)
    #print(len(cones_list[0]))
    for j in range(len(cones_list[i])):
      keypoints_cone = []
      for k in range(7):
        x = predictions[curr+j][k][0]*(or_dimensions[i][j][2] - or_dimensions[i][j][0])/48 + or_dimensions[i][j][0]
        y = predictions[curr+j][k][1]*(or_dimensions[i][j][3] - or_dimensions[i][j][1])/64 + or_dimensions[i][j][1]
        #print(x,y)
        
        monoimg[int(y)-4:int(y)+4,int(x)-4:int(x)+4] = np.array([0,255,0])


        keypoints_cone.append([x,y]) 
      keypoints_cone_numpy = np.array(keypoints_cone)
      # saveImage(Image.fromarray(monoimg, 'RGB'), "mono")
      success, rotation_vector, translation_vector = cv2.solvePnP(objp_orange, keypoints_cone_numpy, cameraMatrix, distCoeffs)
      pnp_image.append([success, rotation_vector, translation_vector])

    return pnp_image

def pipe(img):

    modelpath = "../models/yolov5s6.pt"
    keypoints_modelpath = "../models/KeypointsModelComplex.pt"

    objp_orange = np.array([[0, 32.5 ,0],
                 [-4.3, 20.5, 0],
                 [4.3, 20.5, 0],
                 [-5.8, 11.9, 0],
                 [5.8, 11.9, 0],
                 [-7.4, 2.7, 0],
                 [7.4, 2.7, 0]])

    cameraMatrix= np.load('../pnp/cameraMatrix.npy')
    distCoeffs = np.load('../pnp/distCoeffs.npy')

    imgs = [img]

    # Load YOLO
    model = initYOLOModel(modelpath)

    # Get YOLO results
    results = inferenceYOLO(model, imgs, 1280)

    # Load Keypoints Model
    keypoints_model = initKeypoint(keypoints_modelpath)

    cones_list, classes_list, or_dimensions = cropResizeCones(results, imgs)

    keypoints_predictions = runKeypoints(cones_list, keypoints_model, imgs)

    # print(keypoints_predictions)

    final_coordinates = finalCoordinates(cones_list, or_dimensions, keypoints_predictions, imgs, cameraMatrix, distCoeffs, objp_orange, img)

    print(final_coordinates)


def main():
    #imgpath = "/home/vasilis/Projects/Prom/Perception/gfr_00554.jpg"
    monopath = "../data/52310296366_46561cba17_o.jpg"
    #monopath = "../data/mono 500 50.bmp"
    #stereopath = "/home/vasilis/Projects/Prom/Perception/test_images/stereo l 200 0.bmp"
    modelpath = "../models/best.pt"
    keypoints_modelpath = "../models/KeypointsModelComplex.pt"

    objp_orange = np.array([[0, 32.5 ,0],
                 [-4.3, 20.5, 0],
                 [4.3, 20.5, 0],
                 [-5.8, 11.9, 0],
                 [5.8, 11.9, 0],
                 [-7.4, 2.7, 0],
                 [7.4, 2.7, 0]])

    cameraMatrix= np.load('../pnp/cameraMatrix.npy')
    distCoeffs = np.load('../pnp/distCoeffs.npy')

    #img = cv2.imread(imgpath)
    monoimg = cv2.imread(monopath)
    # print(monoimg)
    #stereoimg = cv2.imread(stereopath)
    imgs = [monoimg]
    # Load YOLO
    model = initYOLOModel(modelpath)

    # Get YOLO results
    results = inferenceYOLO(model, imgs, 1280)
    #results.pred[0][0] -> (x,y,x,y,conf,cls)
    print((results.pandas().xyxy))

    #results = non_max_suppression(results, agnostic=True)

    #print(len(results.pred[0]))

    # Load Keypoints Model
    keypoints_model = initKeypoint(keypoints_modelpath)

    cones_list, classes_list, or_dimensions = cropResizeCones(results, imgs)

    keypoints_predictions = runKeypoints(cones_list, keypoints_model, imgs)

    # print(keypoints_predictions)

    final_coordinates = finalCoordinates(cones_list, or_dimensions, keypoints_predictions, imgs, cameraMatrix, distCoeffs, objp_orange, monoimg)

    #print(final_coordinates)
    





if __name__ == "__main__":
    main()