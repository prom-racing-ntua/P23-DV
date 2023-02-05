from libraries.cameraClass import * #Camera class, daheng api, sys

import cv2
from PIL import Image
import time
import numpy as np
from pathlib import Path
import argparse

import os

def acquisitionParser():
    parser = argparse.ArgumentParser(description='Prom Racing Acquisition Testing')
    parser.add_argument('--frames', type=int, default=1)
    namespace = parser.parse_args()

    return namespace

def main():
    # Basic settings
    roi = (1280,1024)
    exposureTime = 10000

    # Initialize cameras with selected settings
    cameras = initializeCameras(roi=roi, exposureTime=exposureTime)

    # Pass Arguments
    args = acquisitionParser()
    frames = args.frames

    # Acquire Images for selected number of frames
    for i in range(frames):
        for device in cameras:
            device.TriggerCamera()
            numpyImage, frameID = device.AcquireImage()
            cv2.imwrite(f"pictures/camera{device.serialNumber}_frame{frameID}.jpg",numpyImage[...,::-1])

    for device in cameras:
        device.OnClickClose()

if __name__ == "__main__":
    main()