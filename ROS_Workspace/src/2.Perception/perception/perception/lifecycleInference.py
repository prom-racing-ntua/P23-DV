# System related imports
import os
import time
from math import sqrt, pow 

# ROS2 Related Imports
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from custom_msgs.msg import AcquisitionMessage, Perception2Slam, AutoExposureMessage
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

# Homemade Libraries
from .libraries.pipelineFunctions import *
from node_logger.node_logger import *


class InferenceLifecycleNode(Node):
    def __init__(self, yoloModel, smallKeypointsModel):
        super().__init__('inference')
        self.yoloModelPath = yoloModel
        self.smallKeypointsModelPath = smallKeypointsModel
        # Create a log file
        # path = get_package_share_directory("perception")
        # testingLogs = os.path.join(path, "..", "..", "..", "..", "testingLogs")
        # self.fp = open(f'{testingLogs}/Inference_log_file_{int(time.time())}.txt', 'w')
        self.get_logger().warn("\n-- Inference Node Created")
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Inference node needs to configure all the models in the configuration
        phase. Also open a log file if you want to, idk.
        """
        self.publishing = False
        try:
        # Initialize Models
            self.yoloModel = initYOLOModel(self.yoloModelPath, conf=0.7, iou=0.3)
            # self.smallModel, self.largeModel = initKeypoint(self.smallKeypointsModelPath, self.largeKeypointsModelPath)
            self.smallModel = initKeypoint(self.smallKeypointsModelPath)

            # Setup Message Transcoder
            self.bridge = CvBridge()

            # Create Perception/SLAM topic
            self.publisher_ = self.create_lifecycle_publisher(Perception2Slam, 'perception2slam', 10)

            self.right_autoexp_pub = self.create_lifecycle_publisher(AutoExposureMessage, 'autoexposure_right', 10)
            self.left_autoexp_pub  = self.create_lifecycle_publisher(AutoExposureMessage, 'autoexposure_left', 10)

            self.ideal_yellow = [0, 0, 0]   # RGB
            self.ideal_blue   = [0, 0, 0]   # RGB
            self.ideal_brightness = 150     # integer in range[0, 255](incl.)
            self.kp = 1     # Pure number
            self.ki = 0.1   # Pure number
            self.error_integral = [0, 0, 0, 0]    # [0]: color left camera, [1]: color right camera, [2]: brightness left camera, [3]: brightness right camera
            self.integration_frequency = 10

            self.subscription = self.create_subscription(
                AcquisitionMessage,
                'acquisition_topic',
                self.listener_callback,
                10
            )
            #Timestamp logging
            try:
                self.timestamp_log_right = Logger("inference_right")
                self.timestamp_log_left = Logger("inference_left")
                self.log_data = old_Logger("inference")
                self.get_logger().info(self.timestamp_log_right.check())
                self.get_logger().info(self.timestamp_log_left.check())
                self.get_logger().info(self.log_data.check())
            except Exception as e:
                print("mpa"+str(e))

            self.get_logger().warn("\n-- Inference Configured!")
        except Exception as e:
            self.get_logger().error("Inference not configured.\n Error: ", repr(e))

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Start Publishing
        self.publishing = True

        self.get_logger().warn("\n-- Inference Activated!")
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Stop Publishing
        self.publishing = False

        self.get_logger().warn("\n-- Inference Deactivated!")
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Cleanup Models
        self.publishing = False
        del self.yoloModel, self.smallModel
        self.destroy_publisher(self.publisher_)

        self.get_logger().warn("\n-- Inference Un-Configured!")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # Cleanup Models

        if (state.state_id == 1):
            self.get_logger().info("\n-- Inference Shutdown!")
            return TransitionCallbackReturn.SUCCESS

        del self.yoloModel, self.smallModel
        self.destroy_publisher(self.publisher_)

        self.get_logger().info("\n-- Inference Shutdown!")
        return TransitionCallbackReturn.SUCCESS

    def listener_callback(self, msg):
        start_time = self.get_clock().now().nanoseconds / 10**6
        if not self.publishing:
            return
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
            start_time2 = self.get_clock().now().nanoseconds / 10**6
            inferenceTiming = time.time()
            results, inferenceTime = inferenceYOLO(model=self.yoloModel, img=image, tpu=True) #normally True
            yolo_t = self.get_clock().now().nanoseconds / 10**6 - start_time2
            # self.get_logger().info("YOLO time = {:.3f}\n".format(yolo_t))
            # self.get_logger().info(f"{cameraOrientation} results: {len(results)}")
            if results.size == 0:
                self.get_logger().info(f"No cones found from {cameraOrientation} camera")
            else:
                smallConesList, classesList, croppedImagesCorners = cropResizeCones(results, image, 3)
                keypointsPredictions, inf_t = runKeypoints(smallConesList, self.smallModel)
                finalCoords, classesList = finalCoordinates(cameraOrientation, classesList, croppedImagesCorners, keypointsPredictions, 0)
                try:
                    # This sometimes throws an error,don't know why
                    rangeList, thetaList = zip(*finalCoords)
                except ValueError:
                    self.get_logger().error(f"Keypoints could not be determined from {cameraOrientation} camera")
                    return
                
                # Give autoexposure feedback to acquisition
                if len(smallConesList)>2:
                    self.find_autoexposure(cameraOrientation.lower(), smallConesList, classesList)
                else:
                    self.find_autoexposure(cameraOrientation.lower(), original_image = image)

                
                # Send message to SLAM Node
                perception2slam_msg = Perception2Slam()
                perception2slam_msg.global_index = globalIndex
                perception2slam_msg.class_list = [int(a) for a in classesList]
                perception2slam_msg.theta_list = list(thetaList)
                perception2slam_msg.range_list = list(rangeList)

                pub_time_1 = self.get_clock().now().nanoseconds / 10**6
                self.publisher_.publish(perception2slam_msg)
                pub_time_2 = self.get_clock().now().nanoseconds / 10**6

                #Timestamp logging
                if cameraOrientation=="right" or cameraOrientation=="Right":
                    self.timestamp_log_right(start_time, 0, globalIndex)
                    self.timestamp_log_right((pub_time_1 + pub_time_2) / 2, 1, globalIndex)
                elif cameraOrientation=="left" or cameraOrientation=="Left":
                    self.timestamp_log_left(start_time, 0, globalIndex)
                    self.timestamp_log_left((pub_time_1 + pub_time_2) / 2, 1, globalIndex)
                else:
                    self.get_logger().info("Logger error: unknown camera orientation: {:s}".format(cameraOrientation))

                # Log inference time
                inferenceTiming = (time.time() - inferenceTiming)*1000.0 #Inference time in ms
                # self.get_logger().info(f"Inference time is: {inferenceTime} ms")
                self.log_data(f'GlobalIndex: {globalIndex} cameraOrientation: {cameraOrientation} InferenceTime: {inferenceTiming}')


    def find_autoexposure(self, orientation, conesList = None, classesList = None, original_image = None) :
        message = AutoExposureMessage()

        if conesList is not None and classesList is not None:
            message.has_cones = True

            no_of_yellow = np.count_nonzero(np.array(classesList) == 0)
            no_of_blue   = np.count_nonzero(np.array(classesList) == 1)
            total_cones = no_of_blue + no_of_yellow

            yellows = []
            blues   = []


            for i in range(len(classesList)):
                if classesList[i] not in [0, 1]:
                    continue

                masked_image = create_mask(conesList[i], classesList[i])

                pixels = [element for row in masked_image for element in row if np.any(np.array(element))!=0]
                pixels = np.array(pixels)
                average_color = [np.average(x) for x in pixels.transpose()]

                (yellows if classesList[i] == 0 else blues).append(average_color)

            avg_yellow = np.average(np.array(yellows))
            avg_yellow_brightness = max(avg_yellow)
            avg_blue = np.average(np.array(blues))
            avg_blue_brightness   = max(avg_blue)

            yellow_error = self.ideal_yellow[2] - avg_yellow_brightness
            blue_error = self.ideal_blue[2] - avg_blue_brightness
            
            error = (yellow_error * no_of_yellow + blue_error * no_of_blue) / total_cones

            message.brightness = (avg_yellow_brightness * no_of_yellow + avg_blue_brightness * no_of_blue) / total_cones

            if orientation == 'left':
                self.error_integral[0] += error / self.integration_frequency

                correction = error * self.kp + self.error_integral[0] * self.ki

                message.delta_exposure = np.int32(correction)

                if abs(correction) < 50: # microseconds
                    return
                else:
                    self.left_autoexp_pub.publish(message)

            if orientation == 'right':
                self.error_integral[1] += error / self.integration_frequency

                correction = error * self.kp + self.error_integral[1] * self.ki

                message.delta_exposure = np.int32(correction)

                if abs(correction) < 50: # microseconds
                    return
                else:
                    self.right_autoexp_pub.publish(message)

        elif original_image is not None:
            message.has_cones = False

            b_pix = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)[:, :, 2]
            brightness = np.average(np.array(b_pix))

            message.brightness = brightness

            error = self.ideal_brightness - brightness

            if orientation == 'left':
                self.error_integral[2] += error / self.integration_frequency

                correction = error * self.kp + self.error_integral[2] * self.ki

                message.delta_exposure = np.int32(correction)

                if abs(correction) < 50: # microseconds
                    return
                else:
                    self.left_autoexp_pub.publish(message)

            if orientation == 'right':
                self.error_integral[3] += error / self.integration_frequency

                correction = error * self.kp + self.error_integral[3] * self.ki

                message.delta_exposure = np.int32(correction)

                if abs(correction) < 50: # microseconds
                    return
                else:
                    self.right_autoexp_pub.publish(message)

        else:
            self.get_logger().warn(f'Autoexposure algorithm has been given neither cones nor the original image. Plz fix!!!')

    
def main(args=None):
    rclpy.init(args=args)
    
    path = get_package_share_directory("perception")
    models = os.path.join(path,"models")
    # EdgeTPU YOLO
    yolov5_edgetpu_model_path = f"{models}/yolov5n6_640_edgetpu.tflite"
    # yolov5_edgetpu_model_path = f"{models}/yolov5n6-int8.tflite"
    # Yolo v7
    yolov7_model_path = f"{models}/yolov7.pt"
    # Medium Yolo v5
    yolov5m_model_path = f"{models}/yolov5m6.pt"
    # Small Yolo v5
    yolov5s_model_path = f"{models}/yolov5s6.pt"
    # Small Keypoints Parh
    # smallKeypointsModelPath = f"{models}/Res4NetNoBNMSEAugmSize16.xml"
    smallKeypointsModelPath = f"{models}/Res4NetNoBNMSEAugmSize16.pt"
    # Large Keypoints dated 17/1/2023
    # largeKeypointsModelPath = f"{models}/largeKeypoints17012023.pt"
    # Spin inference node
    inference_node = InferenceLifecycleNode(yoloModel=yolov5_edgetpu_model_path, smallKeypointsModel=smallKeypointsModelPath)
    executor = MultiThreadedExecutor(num_threads=3)
    
    try:
        rclpy.spin(inference_node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            inference_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
