# System Related Imports
import sys
import os
from pathlib import Path
import time

# Perception/Acquisition Related Imports
import cv2
import gxipy as gx

# ROS2 Related imports
import rclpy
from perception_msgs.msg import AcquisitionMessage
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

# Homemade Libraries
sys.path.append("/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception")
from cameraClass import Camera
from cnn import *
from pipe import *

def getEpoch():
    return float(time.time())

class AcquisitionNode(Node):
    def __init__(self,runDirPath:str):
        super().__init__('acquisition_node')
        timer_period = 0.1 # seconds (10Hz)

        # Init Devices
        device_manager = gx.DeviceManager()

        # Get Device info - see if cameras are connected
        dev_num, dev_info_list = device_manager.update_device_list()
        if dev_num == 0:
            print("No Devices Found")
            sys.exit(2)

        # Get Parameters from launch file
        self.declare_parameter('serialNumber', '')
        self.serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value

        # TODO: Add parameters from launch file to control exposure time, ROI e.t.c.
        # TODO: O kwdikas einai GTP, den kserw KAN pws katafera na kanw toses malakies, tha ton kanw
        # refactor
        # Initialize camera class
        if (self.serialNumber == "KE0220030137"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="left")
        elif (self.serialNumber == "KE0220040196"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="right")
        elif (self.serialNumber == "KE0220030138"):
            self.camera = Camera((1280,1024), self.serialNumber, orientation="center")
        else:
            self.get_logger().info(f"Found no camera, exiting")
            sys.exit(3)

        # Open Camera and set settings
        self.camera.OnClickOpen()
        self.camera.SetSettings()

        # Setup Publisher
        self.bridge = CvBridge()  #This is used to pass images as ros msgs
        self.publisher_ = self.create_publisher(AcquisitionMessage, 'acquisition_topic', 10)

        # This is used for logging purposes
        self.runDirPath = runDirPath

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Trigger camera and acquire image
        self.camera.TriggerCamera()
        timestamp = getEpoch()
        numpyImage, frameID = self.camera.AcquireImage()

        # Save Image to Run folder
        cv2.imwrite(f"{self.runDirPath}/{self.camera.orientation}_{timestamp}_{frameID}.jpg" ,numpyImage)
        self.get_logger().info(f"Just saved frame {frameID} from camera {self.camera.orientation}")

        # # Send Image to Perception Node
        # msg = AcquisitionMessage()
        # msg.image = self.bridge.cv2_to_imgmsg(numpyImage, encoding="passthrough")
        # msg.camera_orientation = self.camera.orientation
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishng image {frameID} from camera {self.camera.orientation}')

        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    print("Start Camera Node")

    # Create directory to save photos in
    homepath = os.getenv('HOME')
    runTime = int(getEpoch())
    runDirPath = f"{homepath}/PerceptionRuns/Run{runTime}"
    Path(runDirPath).mkdir(parents=True, exist_ok=True)
    print(f"Created Run Directory: {runDirPath}")
    
    perception_handler = AcquisitionNode(runDirPath=runDirPath)
    print("Spin Node")
    rclpy.spin(perception_handler)

    perception_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()