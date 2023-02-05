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
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

# Homemade Libraries
from .libraries.cameraClass import Camera

def getEpoch():
    return float(time.time())

class AcquisitionNode(Node):
    def __init__(self):
        super().__init__('saltas_acquisition_node')

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

        self.get_logger().info(f"{serialNumber}, {orientation}, {exposureTime}, {ROIx}, {ROIy}, {expectedGrayValue}, {autoExposure}")

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

        # Setup saltas clock node
        self.subscription = self.create_subscription(
            NodeSync,
            'saltas_clock',
            self.saltas_callback,
            10
        )

    def saltas_callback(self, msg):
        # Trigger camera and acquire image
        trigger = msg.exec_perception
        if (trigger):
            global_index = msg.global_index            
            self.camera.TriggerCamera()
            numpyImage = self.camera.AcquireImage()

            # Send Image to Perception Node
            imageMessage = AcquisitionMessage()
            imageMessage.global_index = global_index
            imageMessage.image = self.bridge.cv2_to_imgmsg(numpyImage, encoding="passthrough")
            imageMessage.camera_orientation = self.camera.orientation
            self.publisher_.publish(imageMessage)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    # Spin Acquisition Node
    perception_handler = AcquisitionNode()
    executor = SingleThreadedExecutor()
    try:
        rclpy.spin(perception_handler, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Close cameras, needs to be tested THOUROUGHLY!!
        device_manager = gx.DeviceManager()
        dev_num, dev_info_list = device_manager.update_device_list()
        for i in range(dev_num):
            devSN = dev_info_list[i].get("sn")
            camera = device_manager.open_device_by_sn(devSN)
            camera.close_device()
    finally:
        perception_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()