import json
import sys
from datetime import datetime
import os
from pathlib import Path

import numpy as np
import pandas as pd

import cv2
import gxipy as gx
import rclpy
import torch
import torch.optim as optim
import torchvision

from PIL import Image
from rclpy.node import Node
from torch.utils.data import DataLoader, Dataset, SubsetRandomSampler

sys.path.append("/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception")
from acquisition import *
from callback import *
from camera_control import *
from cnn import *
from pipe import *
from helpers import *

from enum import Enum



class PerceptionHandler(Node):
    def __init__(self, camera, yoloModelPath:str, keypointsModelPath:str, runDirPath:str):
        super().__init__('perception_node')
        timer_period = 0.1  # seconds (10Hz)
        self.camera = camera

        self.runDirPath = runDirPath

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # self.yoloModel = initYOLOModel(yoloModelPath)
        # self.keypointsModel = initKeypoint(keypointsModelPath)


    def timer_callback(self):
        # Trigger camera and acquire image
        sendTriggerCommand(self.camera)

        before = datetime.now()
        (numpyImage, _, frameID) = receiveAndConvertImage(self.cameras[0])
        after = datetime.now()
        # self.get_logger().info(f"Time before and after: {before} and {after}")
        cv2.imwrite(f"{self.runDirPath}/{self.camera.orientation}_{getEpoch()}_{frameID}.jpg" ,numpyImage)
        # self.get_logger().info(f"Just saved frame {frameID} from  camera at time {getEpoch()}")
        # Pipeline Starts Here
        
        self.i += 1
    

# TODO: Finish pipeline function
# def pipeline():

#     results = inferenceYOLO(self.yoloModel, imageList, 1280)

#     if results.pandas().xyxy is None:
#         print("Found no Cones")
#     else:
#         conesList, classesList, originalDimensions = cropResizeCones(results, imageList)
#         keypointsPredictions = runKeypoints(conesList, self.keypointsModel, imageList)
#         # finalCoordinates = finalCoordinates(conesList, originalDimensions, keypointsPredictions, imageList, distCoeffs, objp)

#         # cv2.imwrite(f"/home/vasilis/runs/image{self.i}.jpg", numpy_image)
#         # print(f"Just Saved FrameID: {frame_id}")

#         # ZeroOut list (will explain in the future)
#         imageList.clear()

        
def getEpoch():
    return float(time.time())

def main(args=None):
    rclpy.init(args=args)

    print("Start Perception")

    # Create directory to save photos in
    homepath = os.getenv('HOME')
    runTime = getEpoch()
    runDirPath = f"{homepath}/PerceptionRuns/Run{runTime}"
    Path(runDirPath).mkdir(parents=True, exist_ok=True)
    print(f"Created Run Directory: {runDirPath}")

    # Load models (WIP, do not use yet)
    # TODO: Implement Pipeline for real time perception
    yoloModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/yolov5s6.pt"
    keypointsModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/KeypointsModelComplex.pt"

    # Init Devices
    device_manager = gx.DeviceManager()

    # Get Device info
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("No Devices Found")
        sys.exit(2)

    # Create camera classes, set orientation and put them in an array
    cameras = []
    for i in range(dev_num):
        devSN = dev_info_list[i].get("sn")
        if (devSN == "KE0220040196"):
            cam = Camera((1280,1024), devSN, orientation="left")
        elif (devSN == "KE0220030137"):
            cam = Camera((1280,1024), devSN, orientation="right")
        elif (devSN == "KE0220030138"):
            cam = Camera((1280,1024), devSN, orientation="center")
        else:
            cam = Camera((1280,1024), devSN, orientation="random")
        cameras.append(cam)
    
    # Open and setup cameras found in array
    for camera in cameras:
        camera.OnClickOpen()
        camera.SetSettings()

    perception_handler = PerceptionHandler(cameras=cameras, yoloModelPath = yoloModelPath, keypointsModelPath = keypointsModelPath, runDirPath=runDirPath)
    print("Spin Node")
    rclpy.spin(perception_handler)

    perception_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()