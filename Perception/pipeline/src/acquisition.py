# Imports
# Daheng API
import gxipy as gx

# General System Libraries
import sys
import time
from datetime import datetime

# Image Processing Libraries 
from PIL import Image
import torch

# Homemade "Libraries"
from helpers import *
from callback import *
from camera_control import *



# Get info from device manager

# Open Cameras

# Set camera settings (ROI, AutoWhiteBalance, Gain, e.t.c.)

# Set Acquisition Mode (Trigger or Continuous) Target FPS should be 10FPS

# Register Callback Function (found in callback.py)

# Start Acquisition

# If mode is "Trigger" then get trigger signal from ROS 

# If mode is "Continuous" then get in a while(1) loop 

# Stop Acquisition

# Close cameras

