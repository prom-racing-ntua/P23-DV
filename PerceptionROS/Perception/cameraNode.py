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

device_manager = gx.DeviceManager()

class PerceptionHandler(Node):
    def __init__(self, yoloModelPath:str, keypointsModelPath:str, runDirPath:str):
        super().__init__('perception_node')
        timer_period = 0.1  # seconds (10Hz)

        # Init Devices
        device_manager = gx.DeviceManager()

        # Get Device info
        dev_num, dev_info_list = device_manager.update_device_list()
        if dev_num == 0:
            print("No Devices Found")
            sys.exit(2)

        # Get Serial Number from launch file
        self.declare_parameter('serialNumber', '')
        self.serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value

        # Initialize camera class
        if (self.serialNumber == "KE0220040196"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="left")
        elif (self.serialNumber == "KE0220030137"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="right")
        elif (self.serialNumber == "KE0220030138"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="center")
        else:
            self.get_logger().info(f"Found no camera, exiting")
            sys.exit(3)            


        # Open Camera and set settings
        self.camera.OnClickOpen()
        self.camera.SetSettings()

        self.runDirPath = runDirPath

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # self.yoloModel = initYOLOModel(yoloModelPath)
        # self.keypointsModel = initKeypoint(keypointsModelPath)

    def timer_callback(self):
        # Trigger camera and acquire image
        self.camera.TriggerCamera()
        (numpyImage, _, frameID) = receiveAndConvertImage(self.camera.cam)
        # Save Image to Run folder
        cv2.imwrite(f"{self.runDirPath}/{self.camera.orientation}_{getEpoch()}_{frameID}.jpg" ,numpyImage)
        self.i += 1
        
def getEpoch():
    return float(time.time())

def main(args=None):
    rclpy.init(args=args)

    print("Start Perception")

    # Create directory to save photos in
    homepath = os.getenv('HOME')
    runTime = int(getEpoch())
    runDirPath = f"{homepath}/PerceptionRuns/Run{runTime}"
    Path(runDirPath).mkdir(parents=True, exist_ok=True)
    print(f"Created Run Directory: {runDirPath}")

    # Load models (WIP, do not use yet)
    # TODO: Implement Pipeline for real time perception
    yoloModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/yolov5s6.pt"
    keypointsModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/KeypointsModelComplex.pt"
    
    perception_handler = PerceptionHandler(yoloModelPath = yoloModelPath, keypointsModelPath = keypointsModelPath, runDirPath=runDirPath)
    print("Spin Node")
    rclpy.spin(perception_handler)

    perception_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()