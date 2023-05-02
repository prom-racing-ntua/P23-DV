# System related imports
import os
import time

# ROS2 Related Imports
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from custom_msgs.msg import AcquisitionMessage, Perception2Slam
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# Homemade Libraries
from .libraries.pipelineFunctions import *

class InferenceLifecycleNode(Node):
    def __init__(self, yoloModel, smallKeypointsModel, largeKeypointsModel):
        super().__init__('inference')
        self.yoloModelPath = yoloModel
        self.smallKeypointsModelPath = smallKeypointsModel
        self.largeKeypointsModelPath = largeKeypointsModel
        # Create a log file
        self.fp = open(f'testingLogs/Inference_log_file_{int(time.time())}.txt', 'w')
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Inference node needs to configure all the models in the configuration
        phase. Also open a log file if you want to idk.
        """

        # Create Perception/SLAM topic
        self.publisher_ = self.create_lifecycle_publisher(Perception2Slam, 'perception2slam', 10)

        # Initialize Models
        self.yoloModel = initYOLOModel(self.yoloModelPath, conf=0.75, iou=0.45)
        self.smallModel, self.largeModel = initKeypoint(self.smallKeypointsModelPath, self.largeKeypointsModelPath)

        # Setup Message Transcoder
        self.bridge = CvBridge()

        self.get_logger().info("Inference Configuration Complete")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Subscribe to acquisition topic
        self.subscription = self.create_subscription(
            AcquisitionMessage,
            'acquisition_topic',
            self.listener_callback,
            10
        )

        self.get_logger().info("Inference Activation Complete")
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Stop Receiving frames from Acquisition Nodes
        self.destroy_subscription(self.subscription)

        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Cleanup Models
        del self.yoloModel, self.smallModel, self.largeModel
        # Delete Publisher
        self.destroy_lifecycle_publisher(self.publisher_)

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # Cleanup Models
        del self.yoloModel, self.smallModel, self.largeModel
        # Delete Publisher
        self.destroy_lifecycle_publisher(self.publisher_)

        return TransitionCallbackReturn.SUCCESS


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
            if results.empty:
                self.get_logger().info(f"No cones found from {cameraOrientation} camera")
            else:
                smallConesList, largeConesList, classesList, croppedImagesCorners = cropResizeCones(results, image, 3)
                keypointsPredictions = runKeypoints(smallConesList, largeConesList, self.smallModel, self.largeModel)
                finalCoords = finalCoordinates(cameraOrientation, classesList, croppedImagesCorners, keypointsPredictions, 0, image)
                rangeList, thetaList = zip(*finalCoords) # Idea from Alex T(s)afos

                # Send message to SLAM Node
                perception2slam_msg = Perception2Slam()
                perception2slam_msg.global_index = globalIndex
                perception2slam_msg.class_list = list(classesList)
                perception2slam_msg.theta_list = list(thetaList)
                perception2slam_msg.range_list = list(rangeList)
                self.publisher_.publish(perception2slam_msg)   

                # Log inference time
                inferenceTiming = (time.time() - inferenceTiming)*1000.0 #Inference time in ms
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
    
    # Spin inference node
    inference_node = InferenceLifecycleNode(yoloModel=yolov5m_model_path, smallKeypointsModel=smallKeypointsModelPath, largeKeypointsModel=largeKeypointsModelPath)
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
