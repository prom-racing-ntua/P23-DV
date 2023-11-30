# Python Imports
import numpy as np
from math import atan2, sqrt, asin
# Rclpy Import
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
# ROS messages import
from vectornav_msgs.msg import ImuGroup


class RotationCompensation(Node):
    '''
    This node is used to calculate the euler angles of the two vectornav units. Before running this node make sure both
    sensors are configured properly and are running. They should be placed correctly on the position you want to measure them.

    Run the node using the command:
        ros2 run rotation_compensation run
    The output will be printed in the terminal and the node will then shutdown.
    ---
        param: sampling_time
    This parameter determines the period of time during witch the gravitational acceleration will be measured and averaged
    out. Set this parameter through the terminal with:
        ros2 run rotation_compensation run --ros-args -p sampling_time:="t"
    ---
    the math -> http://cache-uat.freescale.com/files/sensors/doc/app_note/AN3461.pdf (docs pending...)
    '''
    def __init__(self) -> None:
        super().__init__("rotation_compensation_node")
        self.sampling_time = self.declare_parameter("sampling_time", 240).value
        
        self.vn_200_average_accelerations = np.empty((1,3))
        self.vn_300_average_accelerations = np.empty((1,3))

        self.vn_200_sub = self.create_subscription(ImuGroup, "/vn_200/raw/imu", self.vn_200_callback, 10)
        self.vn_300_sub = self.create_subscription(ImuGroup, "/vn_300/raw/imu", self.vn_300_callback, 10)

        self.updater = self.create_timer(5, self.updater_callback)

        # Rows represent the sensors (vn200 - vn300) and columns angles (roll-pitch)
        self.euler_angles = np.zeros((2,2))
        self.angles_send = False
    def updater_callback(self):
        self.get_logger().info("{:.2f}.\ttotal = {:.4f}".format(self.get_clock().now().seconds_nanoseconds()[0], (self.init_time + self.sampling_time)))
    def vn_200_callback(self, msg) -> None:
        # if this is the first measurement set the initialization time
        if self.vn_200_average_accelerations.shape[0] == 1:
            self.init_time = self.get_clock().now().seconds_nanoseconds()[0]
        # add the new measurements to the array
        measurements = np.array([msg.accel.x, msg.accel.y, msg.accel.z])
        self.vn_200_average_accelerations = np.vstack((self.vn_200_average_accelerations, measurements))
        # check if sampling time is over
        if self.get_clock().now().seconds_nanoseconds()[0] >= (self.init_time + self.sampling_time):
            self.calculate_average()

    def vn_300_callback(self, msg) -> None:
        # if this is the first measurement set the initialization time
        if self.vn_200_average_accelerations.shape[0] == 1:
            self.init_time = self.get_clock().now().seconds_nanoseconds()[0]
        # add the new measurements to the array
        measurements = np.array([msg.accel.x, msg.accel.y, msg.accel.z])
        self.vn_300_average_accelerations = np.vstack((self.vn_300_average_accelerations, measurements))
        # check if sampling time is over
        if self.get_clock().now().seconds_nanoseconds()[0] >= (self.init_time + self.sampling_time):
            self.calculate_average()

    def calculate_average(self) -> None:
        self.destroy_subscription(self.vn_200_sub)
        self.destroy_subscription(self.vn_300_sub)

        # Get the average of the measurements on each axis and then calculate the roll and pitch euler angles
        a_x = -np.average(self.vn_200_average_accelerations[1:,0])
        a_y = -np.average(self.vn_200_average_accelerations[1:,1])
        a_z = -np.average(self.vn_200_average_accelerations[1:,2])
        self.euler_angles[0,0] = atan2(a_y, a_z)
        self.euler_angles[0,1] = atan2(-a_x, sqrt(a_y**2 + a_z**2))

        a_x = -np.average(self.vn_300_average_accelerations[1:,0])
        a_y = -np.average(self.vn_300_average_accelerations[1:,1])
        a_z = -np.average(self.vn_300_average_accelerations[1:,2])
        self.euler_angles[1,0] = atan2(a_y, a_z)
        self.euler_angles[1,1] = atan2(-a_x, sqrt(a_y**2 + a_z**2))

        self.get_logger().info(f"Euler angles of IMU estimated at {np.rad2deg(self.euler_angles[0,0])} deg roll and {np.rad2deg(self.euler_angles[0,1])} deg pitch")
        self.get_logger().info(f"Euler angles of INS estimated at {np.rad2deg(self.euler_angles[1,0])} deg roll and {np.rad2deg(self.euler_angles[1,1])} deg pitch")

        # Shutdown the node
        self.get_logger().info("Shuting down rotation compensation node...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RotationCompensation()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        get_logger('rclpy').info("Shuting down rotation compensation node...")
        node.destroy_node()
        rclpy.shutdown()