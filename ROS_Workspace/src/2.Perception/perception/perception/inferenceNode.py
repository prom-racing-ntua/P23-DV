# System related imports
import os
import time

# ROS2 Related Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor

from custom_msgs.msg import AcquisitionMessage, Perception2Slam
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# Homemade Libraries
from .libraries.pipelineFunctions import *

class InferenceNode(Node):
    def __init__(self, yoloModel, smallKeypointsModel, largeKeypointsModel):
        super().__init__('inference_node')

        # Subscribe to acquisition topic
        self.subscription = self.create_subscription(
            AcquisitionMessage,
            'acquisition_topic',
            self.listener_callback,
            10
        )

        # Create Perception/SLAM topic
        self.publisher_ = self.create_publisher(Perception2Slam, 'perception2slam_topic', 10)

        # Models
        self.yoloModel = yoloModel
        self.smallKeypointsModel = smallKeypointsModel
        self.largeKeypointsModel = largeKeypointsModel

        self.fp = open(f'testingLogs/Inference_log_file_{int(time.time())}.txt', 'w')

        # Setup Message Transcoder
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Get data from message
            globalIndex = msg.global_index
            cameraOrientation = msg.camera_orientation
            image = self.bridge.imgmsg_to_cv2(msg.image, "passthrough")
        except CvBridgeError as bridgeError:
            # Print error if image conversion was not succseful
            self.get_logger().info(f"Failed to get image from acquisition node {cameraOrientation}, on Index {globalIndex}")

        else:
            # Perform Perception Pipeline
            inferenceTiming = time.time()
            results = inferenceYOLO(self.yoloModel, image, 640)
            # self.get_logger().info(f"YOLO Time: {(time.time() - inferenceTiming)*1000.0}")
            if results.empty:
                self.get_logger().info(f"No cones found from {cameraOrientation} camera")
            else:
                smallConesList, largeConesList, classesList, croppedImagesCorners = cropResizeCones(results, image, 500, 600, 3)
                keypointsTiming = time.time()
                keypointsPredictions = runKeypoints(smallConesList, largeConesList, self.smallKeypointsModel, self.largeKeypointsModel)
                finalCoords = finalCoordinates(cameraOrientation, classesList, croppedImagesCorners, keypointsPredictions, 0, image)
                rangeList, thetaList = zip(*finalCoords) # Idea from Alex T(s)afos
                # self.get_logger().info(f"Keypoints Time: {(time.time() - keypointsTiming)*1000.0}")

                # Send message to SLAM Node
                perception2slam_msg = Perception2Slam()
                perception2slam_msg.global_index = globalIndex
                perception2slam_msg.class_list = list(classesList)
                perception2slam_msg.theta_list = list(thetaList)
                perception2slam_msg.range_list = list(rangeList)
                self.publisher_.publish(perception2slam_msg)   

                # Log inference time
                inferenceTiming = (time.time() - inferenceTiming)*1000.0 #Inference time in ms
                # self.get_logger().info(f"Inference Time: {inferenceTiming}")
                self.fp.write(f'GlobalIndex: {globalIndex} cameraOrientation: {cameraOrientation} InferenceTime: {inferenceTiming}')
    
def main(args=None):
    rclpy.init(args=args)
    
    path = get_package_share_directory("perception")
    models = os.path.join(path,"models")

    # Yolo v7
    yolov7_model_path = f"{models}/yolov7.pt"
    # Medium Yolo v5
    yolov5m_model_path = f"{models}/yolov5m6.pt"
    # Small Yolo v5
    yolov5s_model_path = f"{models}/yolov5s6.pt"
    
    # Small Keypoints Parh
    smallKeypointsModelPath = f"{models}/vggv3strip2.pt"
    # Large Keypoints dated 17/1/2023
    largeKeypointsModelPath = f"{models}/largeKeypoints17012023.pt"

    # Initialize Models
    yoloModel = initYOLOModel(yolov5m_model_path, conf=0.75, iou=0.45)
    smallModel, largeModel = initKeypoint(smallKeypointsModelPath, largeKeypointsModelPath)
    
    # Spin inference node
    inference_node = InferenceNode(yoloModel=yoloModel, smallKeypointsModel=smallModel, largeKeypointsModel=largeModel)
    executor = MultiThreadedExecutor(num_threads=3)

    try:
        rclpy.spin(inference_node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        inference_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
