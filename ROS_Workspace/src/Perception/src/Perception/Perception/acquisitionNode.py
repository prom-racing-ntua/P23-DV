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
from .libraries.cameraClass import Camera
from .libraries.cnn import *
from .libraries.pipe import *

def getEpoch():
    return float(time.time())

class AcquisitionNode(Node):
    def __init__(self):
        super().__init__('acquisition_node')

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
            ('ROIx', 1280),
            ('ROIy', 1024),
            ('nodeTimer', 0.1) 
            ]
        )
        # By default, nodeTimer is set to 0.1seconds (10Hz), this timer is configurable in the acquisition_params.yaml file
        # Get Parameters from launch file
        serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value
        orientation = self.get_parameter('orientation').get_parameter_value().string_value
        exposureTime = self.get_parameter('exposureTime').get_parameter_value().integer_value
        ROIx = self.get_parameter('ROIx').get_parameter_value().integer_value
        ROIy = self.get_parameter('ROIy').get_parameter_value().integer_value
        nodeTimer = self.get_parameter('nodeTimer').get_parameter_value().double_value

        self.get_logger().info(f"{serialNumber}, {orientation}, {exposureTime}, {ROIx}, {ROIy}, {nodeTimer}")

        # Initialize camera class
        self.camera = Camera(resolution=(ROIx, ROIy), serialNumber=serialNumber,
                                orientation=orientation, exposureTime=exposureTime)

        # Open Camera and set settings
        self.camera.OnClickOpen()
        self.camera.SetSettings()

        # Setup Publisher
        self.bridge = CvBridge()  #This is used to pass images as ros msgs
        self.publisher_ = self.create_publisher(AcquisitionMessage, 'acquisition_topic', 10)

        self.timer = self.create_timer(nodeTimer, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Trigger camera and acquire image
        self.camera.TriggerCamera()
        timestamp = getEpoch()
        numpyImage, frameID = self.camera.AcquireImage()

        # # Send Image to Perception Node
        msg = AcquisitionMessage()
        msg.image = self.bridge.cv2_to_imgmsg(numpyImage, encoding="passthrough")
        msg.camera_orientation = self.camera.orientation
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishng image {frameID} from camera {self.camera.orientation}')

        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    # Create directory to save photos in
    # homepath = os.getenv('HOME')
    # runTime = int(getEpoch())
    # runDirPath = f"{homepath}/PerceptionRuns/Run{runTime}"
    # Path(runDirPath).mkdir(parents=True, exist_ok=True)
    # print(f"Created Run Directory: {runDirPath}")
    
    # Spin Acquisition Node
    perception_handler = AcquisitionNode()
    rclpy.spin(perception_handler)

    perception_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()