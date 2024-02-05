from libraries.cameraClass import * #Camera class, daheng api, sys
# from libraries.pipe import *
# from libraries.pipeDebug import *

import cv2
# import torch
from PIL import Image
import time

# import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt
import math

# from torch.utils.data import Dataset
import json
# import torchvision
# from torch.utils.data import SubsetRandomSampler, DataLoader

# import torch.optim as optim
import argparse

import os
import time

def liveStreamImage(title, image):
    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(title, image)

def main():
    cameras = initializeCameras((1024,1024), 500.0 )
    camera = cameras[0]

    start_off = time.time()
    camera.TriggerCamera()
    numpyImage = camera.AcquireImage()
    end_off = time.time()
    print("got image in " + str(end_off-start_off))
    cv2.imshow("Camera with off", numpyImage)
    cv2.waitKey(0)

    start_once = time.time()
    camera.TestAuto()
    end_once = time.time()
    print("set auto in " + str(end_once - start_once))

    start_set = time.time()
    camera.TriggerCamera()
    numpyImage = camera.AcquireImage()
    end_set = time.time()
    print("got image in " + str(end_set-start_set))
    cv2.imshow("Camera after TestAuto", numpyImage)
    cv2.waitKey(0)

    for device in cameras:
        device.OnClickClose()

if __name__ == "__main__":
    main()
    



