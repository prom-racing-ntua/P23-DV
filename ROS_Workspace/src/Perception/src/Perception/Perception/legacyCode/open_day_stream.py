# Daheng API
import gxipy as gx

# General System Libraries
import sys
import time
from datetime import datetime

# Image Processing Libraries 
from PIL import Image
import cv2 as cv
import torch

# Homemade "Libraries"
from helpers import *
from callback import *
from camera_control import *

def main():
    print("Perception Demo")

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
    camera = openViaSN(device_manager, devSN1)

    cameras = [camera]
    model_path = "/home/vasilis/Prom/models/best.pt"
    out = cv.VideoWriter('test.avi', cv.VideoWriter_fourcc(*'DIVX'), 15, (1280,1024))
    model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)
    setTriggerMode(cameras)

    switchAcquisitionAllCameras(cameras, True)

    while(1):
        sendTriggerCommand(cameras[0])
        # time.sleep(0.1)
        img,_,_ = receiveAndConvertImage(cameras[0])
        results = model(img)
        results.show()

        # out.write(results.xyxy[0].numpy())
        # cv.imshow('output',results.xyxy[0].numpy())

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()
    switchAcquisitionAllCameras(cameras, False)
    closeAllCameras(cameras)

if __name__ == "__main__":
    main()

