# Daheng API
import gxipy as gx

# General System Libraries
import sys
import time
from datetime import datetime
import cv2

# Image Processing Libraries 
from PIL import Image

# Homemade "Libraries"
from helpers import *
from callback import *
from camera_control import *

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


def main():
    print("Start Perception")

    # Set Mode for Acquisition Method (WIP)
    mode = "Trigger"

    # Init Devices
    device_manager = gx.DeviceManager()

    # Get Device info
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("No Devices Found")
        sys.exit(2)

    # Get serial number of camera (1 camera so far)
    devSN1 = dev_info_list[0].get("sn")

    # Open camera via SN
    camera1 = openViaSN(device_manager, devSN1)

    cameras = [camera1]
    
    # Set Working Mode
    if mode == "Continuous":
        setContAcquisitionMode(cameras, 20.0)

    elif mode == "Trigger":
        setTriggerMode(cameras)

    # Set ROI (max Width, half height)
    # for i in cameras:  
        setCameraROI(cameras[0], offsetY=256, offsetX=0, img_width=1280, img_height=512)
        # setSettings(i)
    
    # Start acquisition
    switchAcquisitionAllCameras(cameras, True)

    if mode == "Trigger":
        for i in range(5):
            sendTriggerCommand(cameras[0])
            before = datetime.now()
            img1 = receiveAndConvertImage(cameras[0])
            after = datetime.now()
            print(f"Before: {before} and After: {after}")            

    elif mode == "Continuous":
        while(1):
            continue

    # End acquisition
    switchAcquisitionAllCameras(cameras, False)
    closeAllCameras(cameras)

if __name__ == "__main__":
    main()
