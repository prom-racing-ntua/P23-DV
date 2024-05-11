# System Related Imports
import sys
import os
from pathlib import Path
import time

# Perception/Acquisition Related Imports
import cv2
import gxipy

# ROS2 Related Imports
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.publisher import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from custom_msgs.msg import NodeSync, AcquisitionMessage, AutoExposureMessage
from cv_bridge import CvBridge, CvBridgeError

from node_logger.node_logger import *

# Homemade Libraries
from .libraries.cameraClass import Camera

def getEpoch():
    return float(time.time())

class AcquisitionLifecycleNode(Node):
    def __init__(self):
        super().__init__('acquisition')
        # Init Devices
        # Get Device info - see if cameras are connected
        device_manager = gxipy.DeviceManager()
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
            ('log_photos', False),
            ('customAutoexposure', False)
            ]
        )
        self.get_logger().warn(f"\n-- Aquisition Node Created")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.publishing = False

            serialNumber = self.get_parameter('serialNumber').get_parameter_value().string_value
            orientation = self.get_parameter('orientation').get_parameter_value().string_value
            exposureTime = self.get_parameter('exposureTime').get_parameter_value().integer_value
            autoExposure = self.get_parameter('autoExposure').get_parameter_value().bool_value
            expectedGrayValue = self.get_parameter('expectedGrayValue').get_parameter_value().integer_value
            ROIx = self.get_parameter('ROIx').get_parameter_value().integer_value
            ROIy = self.get_parameter('ROIy').get_parameter_value().integer_value
            self.log_photos = self.get_parameter("log_photos").get_parameter_value().bool_value
            self.customAutoexposure = self.get_parameter('customAutoexposure').get_parameter_value().bool_value

            if self.customAutoexposure:
                autoExposure = False

            self.get_logger().info(f"{serialNumber}, {orientation}, {exposureTime}, {ROIx}, {ROIy}, {expectedGrayValue}, {autoExposure}, {self.customAutoexposure}")
            self.orientation = orientation
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

            # Create Auto-exposure feedback topic
            if self.customAutoexposure:
                self.feedback_from_inference = self.create_subscription(
                    AutoExposureMessage,
                    f'autoexposure_{orientation.lower()}',
                    self.autoexposure_feedback,
                    10
                )
                self.current_exposure = exposureTime

            # Setup Publisher
            self.bridge = CvBridge()  #This is used to pass images as ros msgs
            self.publisher_ = self.create_publisher(AcquisitionMessage, 'acquisition_topic', 10)

            # Timestamp logging
            # run_idx_file = open("timestamp_logs/run_idx.txt", "r")
            # run_idx = str(int(run_idx_file.read()))
            # run_idx_file.close()
            # self.timestamp_log = open("timestamp_logs/run_" + run_idx + "/"+orientation+"_acquisition_log.txt")

            self.timestamp_log = Logger("acquisition_{:s}".format(orientation))
            self.get_logger().info(self.timestamp_log.check())
            if self.customAutoexposure:
                self.autoexp_timestamp_log: Logger = Logger("autoexposure_{:s}".format(orientation))
                self.get_logger().info(self.autoexp_timestamp_log.check())
            
            if self.log_photos:
                try:
                    self.log_dir = self.timestamp_log.get_run_path()
                    os.mkdir(os.path.join(self.log_dir, f'photos_{orientation}'))
                except Exception as e:
                    self.get_logger().warn(f'Error creating photo {orientation} logging directory: {repr(e)}')

            self.get_logger().warn(f"\n-- Acquisition Configured!")
        except Exception as e:
            self.get_logger().error(f"Error in Acquisition {orientation} configuration", repr(e))
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
        
        self.destroy_subscription(self.subscription)
        self.destroy_publisher(self.publisher_)

        self.camera.cleanupCamera()
        del self.camera, self.bridge

        self.get_logger().info(f"\n-- Acquisition Shutdown!")
        return TransitionCallbackReturn.SUCCESS
    
    def autoexposure_feedback(self, msg: AutoExposureMessage) -> None:
        try:
            if abs(msg.delta_exposure)<1:
                return

            start_time = self.get_clock().now().nanoseconds / 10**6

            
            result = self.camera.SetAutoExposure(self.current_exposure)
            self.current_exposure += msg.delta_exposure if result is None else 0

            # self.get_logger().warn(f'{repr(result) if result is not None else "Success"}, {msg.delta_exposure}, {msg.brightness}, {msg.has_cones}, {self.camera.GetExposure()}')
            
            if result is not None:
                self.get_logger().error(f'Error during setting exposure: {str(result)}')
                self.autoexp_timestamp_log(start_time, 0, 0, [0, msg.delta_exposure, msg.brightness, msg.has_cones, self.current_exposure])
            else:
                self.autoexp_timestamp_log(start_time, 0, 0, [1,msg.delta_exposure, msg.brightness, msg.has_cones, self.current_exposure])
        except:
            self.get_logger().error(f'Error during setting exposure: {str(result)}')
            return
            


    def trigger_callback(self, msg):
        # Trigger camera and acquire image
        if not self.publishing:
            return 
        
        trigger = msg.exec_perception
        if (trigger):
            start_time = self.get_clock().now().nanoseconds / 10**6
            global_index = msg.global_index            
            self.camera.TriggerCamera()
            numpyImage = self.camera.AcquireImage()

            # Send Image to Perception Node
            imageMessage = AcquisitionMessage()
            imageMessage.global_index = global_index
            imageMessage.image = self.bridge.cv2_to_imgmsg(numpyImage, encoding="passthrough")
            imageMessage.camera_orientation = self.camera.orientation

            pub_time_1 = self.get_clock().now().nanoseconds / 10**6
            self.publisher_.publish(imageMessage)
            pub_time_2 = self.get_clock().now().nanoseconds / 10**6


            if self.log_photos and global_index%(4 * 10)==0 and 0:
                try:
                    #path to save the image
                    path = os.path.join(self.log_dir, f'photos_{self.orientation}')
                    #name the image based on the global index and give the path to save
                    path = os.path.join(path, f'image_{global_index}_{self.orientation}.jpg')
                    numpyImage = cv2.cvtColor(numpyImage, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(path, numpyImage)
                except Exception as z:
                    self.get_logger().info("Photo logger error: {:s}".format(repr(z)))
                    self.log_photos = False

            #Timestamp logging
            self.timestamp_log(start_time, 0, global_index)
            self.timestamp_log((pub_time_1 + pub_time_2) / 2, 1, global_index)
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