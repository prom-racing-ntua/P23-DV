# System related imports
import sys

# ROS2 Related Imports
import rclpy
from rclpy.node import Node
from perception_msgs.msg import AcquisitionMessage
from cv_bridge import CvBridge, CvBridgeError

# Perception Related Imports
import cv2
import numpy as np
import math
import json

import torchvision
import torch
from torch.utils.data import Dataset
from torch.utils.data import SubsetRandomSampler, DataLoader
import torch.optim as optim

# Homemade Libraries
from .libraries.cnn import *
from .libraries.pipe import initYOLOModel, inferenceYOLO, initKeypoint, cropResizeCones, runKeypoints, finalCoordinates, pipe

class InferenceNode(Node):
    def __init__(self, models, arrays):
        super().__init__('inference_node')

        # Subscribe to acquisition topic
        self.subscription = self.create_subscription(
            AcquisitionMessage,
            'acquisition_topic',
            self.listener_callback,
            10
        )

        # Create Perception/SLAM topic
        # self.publisher_ = self.create_publisher(Perception2Slam, 'perception2slam_topic', 10)

        # Add models to class
        self.yoloModel = models[0]
        self.smallKeypointsModel = models[1]
        # self.largeKeypointsModel = models[2]

        # Numpy arrays used for functions
        self.cameraMatrix = arrays[0]
        self.distCoeffs = arrays[1]
        self.objp_orange = arrays[2]

        # Setup Message Transcoder
        self.bridge = CvBridge()
        self.i = 0

    def listener_callback(self, msg):
        try:
            # Get data from message
            orientation = msg.camera_orientation
            image = self.bridge.imgmsg_to_cv2(msg.image, "passthrough")
        except CvBridgeError as e:
            # Print error if image conversion was not succseful
            print(e)
        else:
            self.get_logger().info(f'Received image{self.i} looking {orientation}')
            # Perform Perception Pipeline
            images = [image]
            results = inferenceYOLO(self.yoloModel, images, 1280)
            conesList, classesList, originalDimensions = cropResizeCones(results, images)
            keypointsPredictions = runKeypoints(conesList, self.smallKeypointsModel, images)
            final_coordinates = finalCoordinates(conesList, originalDimensions, keypointsPredictions, images, self.cameraMatrix, self.distCoeffs, self.objp_orange)
            self.get_logger().info(f'Printing Final Coordinates {final_coordinates}')

            # Send message to SLAM Node
            # self.publisher_.publish(perception2slam_msg)
    

def main(args=None):
    rclpy.init(args=args)

    # TODO: Setup global project paths
    # Setup Models and NumpyArray paths
    yoloModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/yolov5s6.pt"
    smallKeypointsModelPath = "/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/models/KeypointsModelComplex.pt"
    largeKeypointsModelPath = ""

    cameraMatrix = np.load('/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/cameraMatrix.npy')
    distCoeffs = np.load('/home/vasilis/Projects/Prom/P23-DV-Workspace/src/Perception/Perception/distCoeffs.npy')
    objp_orange = np.array([[0, 32.5 ,0],
                 [-4.3, 20.5, 0],
                 [4.3, 20.5, 0],
                 [-5.8, 11.9, 0],
                 [5.8, 11.9, 0],
                 [-7.4, 2.7, 0],
                 [7.4, 2.7, 0]])

    arrays = [cameraMatrix, distCoeffs, objp_orange]

    # Initialize Models
    yoloModel = initYOLOModel(yoloModelPath, conf=0.25, iou=0.45)
    smallKeypointsModel = initKeypoint(smallKeypointsModelPath)

    models = [yoloModel, smallKeypointsModel]
    
    inference_node = InferenceNode(models = models, arrays=arrays)
    rclpy.spin(inference_node)

    inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()