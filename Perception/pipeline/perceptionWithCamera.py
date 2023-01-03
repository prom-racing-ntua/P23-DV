from libraries.cameraClass import * #Camera class, daheng api, sys
from libraries.pipe import *
from libraries.pipeDebug import *

import cv2
import torch
from PIL import Image
import time

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

def liveStreamImage(title, image):
    cv2.imshow(title, image)

def main():
    cameras = initializeCameras()

    while(1):
        time.sleep(0.05)
        for device in cameras:
            device.TriggerCamera()
            numpyImage, frameID = device.AcquireImage()
            liveStreamImage("Camera Output", numpyImage[...,::-1])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    for device in cameras:
        device.OnClickClose()

if __name__ == "__main__":
    main()
    



