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
from custom_msgs.msg import NodeSync, AcquisitionMessage
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Homemade Libraries
from .libraries.cameraClass import Camera
from .libraries.cnn import *

def getEpoch():
    return float(time.time())

class AcquisitionNode(Node):
    def __init__(self,runDirPath:str):
        super().__init__('acquisition_logger_node')

        timer_period = 0.1 # seconds (10Hz)

        # Init Devices
        device_manager = gx.DeviceManager()

        # Get Device info - see if cameras are connected
        dev_num, dev_info_list = device_manager.update_device_list()
        if dev_num == 0:
            print("No Devices Found")
            sys.exit(2)
        devSN = dev_info_list[0].get("sn")

        self.declare_parameters(
        namespace='',
        parameters=[
        ('serialNumber', devSN),
        ('orientation', 'random'),
        ('exposureTime', 10000),
        ('autoExposure', False),
        ('expectedGrayValue', 50),
        ('ROIx', 1280),
        ('ROIy', 1024),
        ('nodeTimer', 0.1) 
        ]
    )
        # Get Parameters from launch file
        serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value
        orientation = self.get_parameter('orientation').get_parameter_value().string_value
        exposureTime = self.get_parameter('exposureTime').get_parameter_value().integer_value
        autoExposure = self.get_parameter('autoExposure').get_parameter_value().bool_value
        expectedGrayValue = self.get_parameter('expectedGrayValue').get_parameter_value().integer_value
        ROIx = self.get_parameter('ROIx').get_parameter_value().integer_value
        ROIy = self.get_parameter('ROIy').get_parameter_value().integer_value
        nodeTimer = self.get_parameter('nodeTimer').get_parameter_value().double_value

        self.get_logger().info(f"{serialNumber}, {orientation}, {exposureTime}, {ROIx}, {ROIy}, {nodeTimer}")

        # Initialize camera class
        self.camera = Camera(resolution=(ROIx, ROIy), serialNumber=serialNumber,
                                orientation=orientation, exposureTime=exposureTime,
                                autoExposure=autoExposure, expectedGrayValue=expectedGrayValue)

        # Open Camera and set settings
        self.camera.OnClickOpen()
        self.camera.SetSettings()

        # Setup Publisher
        self.bridge = CvBridge()  #This is used to pass images as ros msgs
        self.publisher_ = self.create_publisher(AcquisitionMessage, 'acquisition_topic', 10)

        # This is used for logging purposes
        self.runDirPath = runDirPath

        self.timer = self.create_timer(nodeTimer, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Trigger camera and acquire image
        self.camera.TriggerCamera()
        timestamp = getEpoch()
        numpyImage = self.camera.AcquireImage()

        # Save Image to Run folder
        self.get_logger().warn(f"{numpyImage}")
        print("type is:",type(numpyImage))
        cv2.imwrite(f"{self.runDirPath}/{self.camera.orientation}_{timestamp}_{self.i}.jpg" ,cv2.cvtColor(numpyImage, cv2.COLOR_RGB2BGR))

        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    # Create directory to save photos in
    homepath = os.getenv('HOME')
    print("prin tin get epoch")
    runTime = int(getEpoch())
    share_dir = get_package_share_directory('perception')
    runDirPath = f"{share_dir}/../../../../PerceptionRuns/Run{runTime}"
    Path(runDirPath).mkdir(parents=True, exist_ok=True)
    print(f"Created Run Directory: {runDirPath}")
    
    # Spin Acquisition Node
    perception_handler = AcquisitionNode(runDirPath=runDirPath)
    rclpy.spin(perception_handler)

    perception_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
