# System related imports
import sys
import os

# ROS2 Related Imports
import rclpy
from rclpy.node import Node
from perception_msgs.msg import AcquisitionMessage, Perception2Slam
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# Perception Related Imports
import cv2
import numpy as np
import math
import pandas as pd
import json

import torchvision
import torch
from torch.utils.data import Dataset
from torch.utils.data import SubsetRandomSampler, DataLoader
import torch.optim as optim

# Homemade Libraries
from .libraries.pipelineFunctions import *

class InferenceNode(Node):
    def __init__(self, models):
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

        # Models
        self.yoloModel = models[0]
        self.smallKeypointsModel = models[1]
        self.largeKeypointsModel = models[2]

        # Setup Message Transcoder
        self.bridge = CvBridge()
        self.i = 0

    def listener_callback(self, msg):
        try:
            # Get data from message
            # globalIndex
            cameraOrientation = msg.camera_orientation
            image = self.bridge.imgmsg_to_cv2(msg.image, "passthrough")
        except CvBridgeError as e:
            # Print error if image conversion was not succseful
            print(e)
        else:
            self.get_logger().info(f'Received image {self.i} looking {cameraOrientation}')
            # Perform Perception Pipeline
            results = inferenceYOLO(self.yoloModel, image, 1280)
            if results.pandas().xyxy[0].empty:
                self.get_logger().info("No cones found")
            else:
                conesList, classesList, croppedImagesCorners = cropResizeCones(results, image, 3)
                keypointsPredictions = runKeypoints(conesList, self.smallKeypointsModel)
                finalCoords = finalCoordinates(cameraOrientation, classesList, croppedImagesCorners, keypointsPredictions, 0)
                # Send message to SLAM Node
                perception2slam_msg = Perception2Slam()
                # gloablIndex
                perception2slam_msg.classList = classesList
                # thetaList
                # rangeList
                # self.publisher_.publish(perception2slam_msg)    
    
def main(args=None):
    rclpy.init(args=args)
    
    path = get_package_share_directory("Perception")
    models = os.path.join(path,"models")

    yoloModelPath = f"{models}/yolov5s6.pt"
    smallKeypointsModelPath = f"{models}/KeypointsNet(333).pt"
    largeKeypointsModelPath = f"{models}/largeKeypoints412023.pt"

    # Initialize Models
    yoloModel = initYOLOModel(yoloModelPath, conf=0.25, iou=0.45)
    smallKeypointsModel = initSmallKeypoints(smallKeypointsModelPath)
    largeKeypointsModel = initLargeKeypoints(largeKeypointsModelPath)

    models = [yoloModel, smallKeypointsModel, largeKeypointsModel]
    
    inference_node = InferenceNode(models = models)
    rclpy.spin(inference_node)

    inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()