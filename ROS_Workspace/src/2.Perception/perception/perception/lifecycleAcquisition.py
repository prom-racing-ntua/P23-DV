# System Related Imports
import sys
import os
from pathlib import Path
import time

# Perception/Acquisition Related Imports
import cv2
import gxipy as gx

# ROS2 Related Imports
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.publisher import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from custom_msgs.msg import NodeSync, AcquisitionMessage
from cv_bridge import CvBridge, CvBridgeError

# Homemade Libraries
from .libraries.cameraClass import Camera

def getEpoch():
    return float(time.time())

class AcquisitionLifecycleNode(Node):
    def __init__(self):
        super().__init__('acquisition')
        # Init Devices
        # Get Device info - see if cameras are connected
        device_manager = gx.DeviceManager()
        dev_num, dev_info_list = device_manager.update_device_list()
        if dev_num == 0:
            self.get_logger().error("No Devices Found")
            sys.exit(2)
        devSN = dev_info_list[0].get("sn")
        
        # Get Parameters from launch file 
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
        self.get_logger().warn(f"\n-- Aquisition Node Created")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publishing = False

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

        # Setup saltas clock node
        self.subscription = self.create_subscription(
            NodeSync,
            'saltas_clock',
            self.trigger_callback,
            10
        )

        # Setup Publisher
        self.bridge = CvBridge()  #This is used to pass images as ros msgs
        self.publisher_ = self.create_publisher(AcquisitionMessage, 'acquisition_topic', 10)

        self.get_logger().warn(f"\n-- Acquisition Configured!")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Activate Camera Stream
        self.publishing = True
        self.camera.activateAcquisition()
        
        self.get_logger().warn(f"\n-- Acquisition Activated!")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Deactivate Camera Stream
        self.publishing = False        
        self.camera.deactivateAcquisition()

        self.get_logger().warn(f"\n-- Acquisition Deactivated!")
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.publishing = False
        self.camera.cleanupCamera()

        del self.camera, self.bridge
        self.destroy_publisher(self.publisher_)
        
        self.get_logger().warn(f"\n-- Acquisition Un-Configured!")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if (state.state_id == 1):
            self.get_logger().info(f"\n-- Acquisition Shutdown!")
            return TransitionCallbackReturn.SUCCESS
        
        self.camera.cleanupCamera()

        del self.camera, self.bridge
        self.destroy_publisher(self.publisher_)

        self.get_logger().info(f"\n-- Acquisition Shutdown!")
        return TransitionCallbackReturn.SUCCESS

    def trigger_callback(self, msg):
        # Trigger camera and acquire image
        if not self.publishing:
            return 
        
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
    perception_handler = AcquisitionLifecycleNode()
    executor = SingleThreadedExecutor()
    try:
        rclpy.spin(perception_handler, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            perception_handler.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()