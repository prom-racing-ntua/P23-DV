#import gxipy as gx
import time
from PIL import Image

import math
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch

import cv2
import numpy as np
import torch
import torchvision
import sys

# Image Functions
def saveImage(img, imgName:str):
    img.save(f"{imgName}.jpg")

def showImage(img):
    img.show()

# Triggering Functions
def sendTriggerCommand(camera):
    camera.TriggerSoftware.send_command()

def triggerAllCameras(cameras):
    for i in cameras:
        sendTriggerCommand(i)

# Image Processing Functions
def receiveAndConvertImage(camera):
    """
           Desc:    Converts raw data that it just received to usable image
           Returns: Tuple in the form of (image,timestamp)
    """
    raw_image = camera.data_stream[0].get_image()
    timestamp = raw_image.get_timestamp()
    frame_id = raw_image.get_frame_id()
    if raw_image is None:
        print("Failed to capture image")
    # Convert raw data to rgb
    rgb_image = raw_image.convert("RGB")
    if rgb_image is None:
        print("test")
    # Convert rgb data to numpy array
    numpy_image = rgb_image.get_numpy_array()
    if numpy_image is None:
        print("hihi")
    # Image Processing...
    # img = Image.fromarray(numpy_image, 'RGB')
    return numpy_image, timestamp, frame_id
    
# General Purpose Functions
def nanoToSecond(timestamp:int):
    """For timestamp calculation purposes, timestamp
    is a counter that starts when the device is powered
    on. It's return value is an int in nanoseconds"""
    return float(timestamp/1000000000)