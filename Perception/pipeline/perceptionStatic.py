# Run this script while selecting a folder to output
# everything related to perception
import cv2
import torch
from PIL import Image

import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt
import math

from torch.utils.data import Dataset
import json
import torchvision
from torch.utils.data import SubsetRandomSampler, DataLoader

import torch.optim as optim
import argparse

import os

from libraries.pipe import *
from libraries.pipeDebug import cropResizeConesDebug,runKeypointsDebug, finalCoordinatesDebug

def parser():
    parser = argparse.ArgumentParser(description='Prom Racing Perception Pipeline')
    parser.add_argument('--folder', type=str, default='data')
    namespace = parser.parse_args()

    return namespace

def main():
    # Parse Arguments
    arguments = parser()
    dataFolder = arguments.folder

    ## Load Keypoints and Numpy Arrays
    #Models
    yoloModelPath = f"models/yolov5s6.pt"
    smallKeypointsModelPath = f"models/KeypointsModelComplex.pt"

    yoloModel = initYOLOModel(yoloModelPath)
    smallKeypointsModel = initKeypoint(smallKeypointsModelPath)

    #Numpy Arrays
    cameraMatrix = np.load(f"pnp/cameraMatrix.npy")
    distCoefficients = np.load(f"pnp/distCoeffs.npy") 
    objp_orange = np.array([[0, 32.5 ,0],
                 [-4.3, 20.5, 0],
                 [4.3, 20.5, 0],
                 [-5.8, 11.9, 0],
                 [5.8, 11.9, 0],
                 [-7.4, 2.7, 0],
                 [7.4, 2.7, 0]])
    
    # Run pipeline in every image file:
    folderPath = f"{os.path.dirname(os.path.abspath(dataFolder))}/{dataFolder}"
    Path(f"{folderPath}/predictions").mkdir(parents=True, exist_ok=True)

    files = os.listdir(dataFolder)
    print(files)
    print(len(files))

    for file in os.listdir(dataFolder):
        if (file.endswith(".jpg") or file.endswith(".bmp") or file.endswith(".png")):
            # Create a cones folder (way too many photos, not recommended) If you want the cropped cones photos, then you should also change the cropResizeCones
            # function to cropResizeConesDebug
            # Path(f"{folderPath}/predictions/{file.split('.')[0]}_cones").mkdir(parents=True, exist_ok=True)

            filePath = f"{folderPath}/{file}"

            # Read Image. This weird thing at the end is for color conversion,
            # it says it is faster, idk
            inputImage = cv2.imread(filePath)[...,::-1]
            input = [inputImage]

            # Run inference on yolo model
            results = inferenceYOLO(yoloModel, input, 1280)
            if results.pandas().xyxy[0].empty:
                print("No Cones Found in this image")
            else:
                conesList, classesList, originalDimensions = cropResizeConesDebug(results, input, file,folderPath,input[0])
                keypointsPredictions = runKeypoints(conesList, smallKeypointsModel, input)
                finalCoords = finalCoordinatesDebug(conesList, originalDimensions, keypointsPredictions, input, cameraMatrix, distCoefficients, objp_orange, input[0], file, folderPath)

                print(f"Filename:{file}")
                for i in range(len(classesList[0])):
                    print(f"Cone{i}: Class - {classesList[0][i]}, RotationVector: {finalCoords[i][1]}, TranslationVector: {finalCoords[i][2]}")

if __name__ == "__main__":
    main()