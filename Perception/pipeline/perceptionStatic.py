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

from libraries.pipev2 import *

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
    # smallKeypointsModelPath = f"models/vggv3strip2.pt"
    smallKeypointsModelPath = f"models/KeypointsNet(333).pt"

    yoloModel = initYOLOModel(yoloModelPath)
    smallKeypointsModel = initKeypoint(smallKeypointsModelPath)

    # Run pipeline in every image file:
    folderPath = f"{os.path.dirname(os.path.abspath(dataFolder))}/{dataFolder}"
    Path(f"{folderPath}/predictions").mkdir(parents=True, exist_ok=True)

    files = os.listdir(dataFolder)
    for file in os.listdir(dataFolder):
        if (file.endswith(".jpg") or file.endswith(".bmp") or file.endswith(".png")):

            filePath = f"{folderPath}/{file}"

            # Read Image. This weird thing at the end is for color conversion,
            # it says it is faster, idk
            inputImage = cv2.imread(filePath)[...,::-1]

            # Run inference on yolo model
            results = inferenceYOLO(yoloModel, inputImage, 1280)
            if results.pandas().xyxy[0].empty:
                print("No Cones Found in this image")
            else:
                conesList, classesList, croppedImageCorners = cropResizeCones(results, inputImage, 3)
                keypointsPredictions = runKeypoints(conesList, smallKeypointsModel)
                final_coords = finalCoordinates('left', classesList, croppedImageCorners, keypointsPredictions, 0)
                for i in range(len(classesList)):
                    print(file)
                    print("Class: ", classesList[i], " Range: ", final_coords[i][0], " Theta: ", final_coords[i][1])

if __name__ == "__main__":
    main()