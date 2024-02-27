import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor, ExternalShutdownException
from ament_index_python import get_package_share_directory
import csv
import os
import pandas as pd
import atexit
import time
from rclpy.timer import Timer
from node_logger.node_logger import *


from vectornav_msgs.msg import ImuGroup, InsGroup, GpsGroup, AttitudeGroup
from custom_msgs.msg import WheelSpeed, BrakePressure, SteeringAngle, VelEstimation

FREQUENCY = 10  # Hz

class DataLogger(Node):
    def __init__(self) -> None:
        super().__init__("data_logger")
        self.start_time = int(self.get_clock().now().nanoseconds/10**9)
        self.declare_parameter('verbose', True)
        self._verbose = self.get_parameter('verbose').value

        self.imu_flag = 0
        self.imu_index = 0
        self.ins_flag = 0
        self.ins_index = 0

        self.vn200_log = Logger("vn200")
        self.get_logger().info(self.vn200_log.check())
        self.vn300_log = Logger("vn300")
        self.get_logger().info(self.vn300_log.check())

        sensor_data_profile = QoSPresetProfiles.get_from_short_key("SENSOR_DATA")
        self._vn_200_imu_subscriber = self.create_subscription(ImuGroup, '/vn_200/raw/imu', self.vn_200_imu_callback, sensor_data_profile)
        self._vn_300_ins_subscriber = self.create_subscription(InsGroup, '/vn_300/raw/ins', self.vn_300_ins_callback, sensor_data_profile)
        
    def vn_200_imu_callback(self, msg) -> None:
        meas_time = self.get_clock().now().nanoseconds/10**6
        self.vn200_log(meas_time, 0, self.imu_index, [msg.accel.x,msg.accel.y,msg.accel.z, msg.angularrate.z])

        self.imu_index += 1

    def vn_300_ins_callback(self, msg) -> None:
        meas_time = self.get_clock().now().nanoseconds/10**6
        self.vn300_log(meas_time, 0, self.imu_index, [msg.insstatus.mode,msg.insstatus.gps_heading_ins, msg.insstatus.gps_compass,msg.velbody.x,msg.velbody.y,msg.velbody.z, (msg.velbody.x**2+msg.velbody.y**2+msg.velbody.z**2)**0.5])

        self.ins_index += 1
        

def main(args=None):
    rclpy.init(args=args)
    logger = DataLogger()

    executor = MultiThreadedExecutor(num_threads=2)
    #executor = SingleThreadedExecutor()
    
    try:
        rclpy.spin(logger, executor=executor)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()