# System Related Imports
import sys
import os
from pathlib import Path
import time
import numpy as np

# ROS2 Related imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory


from geometry_msgs.msg import Vector3
from custom_msgs.msg import VelEstimation, RxVehicleSensors, Perception2Slam
from vectornav_msgs.msg import ImuGroup, InsGroup, InsStatus
from node_logger.node_logger import *


class VelocitySimNode(Node):
    def __init__(self) -> None:
        super().__init__('velocity_sim_node')
        self.vectornav_index = 0
        self.sensor_index = 0
        self.perception_index = 0
        run = 78

        self.declare_parameters(
            namespace='',
            parameters=[
            ('vn200_data_file', f'/home/nick/Desktop/DV-Testing-1/run_{run}/vn200_log.txt'),
            ('vn300_data_file', f'/home/nick/Desktop/DV-Testing-1/run_{run}/vn300_log.txt'),
            ('sensor_data_file', f'/home/nick/Desktop/DV-Testing-1/run_{run}/canbus_sensor_log.txt'),
            ('perception_data_file', f'/home/nick/Desktop/DV-Testing-1/run_{run}/perceptionLog.txt'),
            ('sim_relative_frequency', 1)
            ]
        )

        vn200_data_filename  = self.get_parameter('vn200_data_file').get_parameter_value().string_value
        vn300_data_filename  = self.get_parameter('vn300_data_file').get_parameter_value().string_value
        sensor_data_filename = self.get_parameter('sensor_data_file').get_parameter_value().string_value
        perception_data_filename = self.get_parameter('perception_data_file').get_parameter_value().string_value
        relative_frequency = self.get_parameter('sim_relative_frequency').get_parameter_value().integer_value


        # Publisher for the synchronization topic
        self.vn200_publisher = self.create_publisher(ImuGroup, '/vn_200/raw/imu', 10)
        self.vn300_publisher = self.create_publisher(InsGroup, '/vn_300/raw/ins', 10)
        self.motor_publisher = self.create_publisher(RxVehicleSensors, '/canbus/sensor_data', 10)
        self.perception_publisher = self.create_publisher(Perception2Slam, '/perception2slam', 10)

        self.vectornav_clock = self.create_timer(0.025 / relative_frequency, self.vectornav_callback)
        self.motor_clock = self.create_timer(0.05 / relative_frequency, self.sensor_callback)
        # self.perception_clock = self.create_timer(0.1 / relative_frequency, self.perception_callback)

        # Subscriber to velocity estimation results for data analysis
        self.velocity_sub = self.create_subscription(VelEstimation, '/velocity_estimation', self.velocity_callback, 10)

        self.log = Logger('pseudo_velocity')
        self.get_logger().info(self.log.check())

        self.results = [
            [], # v_x
            [], # v_y
            [], # v_yaw
            [], # a_x
            [], # a_y
            [],[],[],[],[],[],[],[],[] # covariances
        ]

        vn200_data_file  = open(vn200_data_filename, 'r')
        vn300_data_file  = open(vn300_data_filename, 'r')
        sensor_data_file = open(sensor_data_filename, 'r')
        perception_data_file = open(perception_data_filename, 'r')

        self.vn200_data = self.get_data_from_file(vn200_data_file)
        self.vn300_data = self.get_data_from_file(vn300_data_file)
        self.sensor_data = self.get_data_from_file(sensor_data_file)
        # self.perception_data = self.get_data_from_file(perception_data_file)
        self.perception_data = [line.split() for line in perception_data_file.readlines()]

        self.end = False

        vn200_t0 = self.vn200_data[0][0][0]
        vn300_t0 = self.vn200_data[0][0][0]
        sensor_t0 = self.sensor_data[0][0][0]
        self.delay = sensor_t0 - min(vn200_t0, vn300_t0)

        # print(np.shape(np.array(self.vn200_data)))

        self.get_logger().info(f'Sim is online')

    def perception_callback(self):
        cam1_index = int(self.perception_data[self.perception_index][0])
        cam1_classes = [int(x) for x in self.perception_data[self.perception_index+1]]
        cam1_ranges = [float(x) for x in self.perception_data[self.perception_index+2]]
        cam1_thetas = [float(x) for x in self.perception_data[self.perception_index+3]]

        cam2_index = int(self.perception_data[self.perception_index+4][0])
        cam2_classes = [int(x) for x in self.perception_data[self.perception_index+5]]
        cam2_ranges = [float(x) for x in self.perception_data[self.perception_index+6]]
        cam2_thetas = [float(x) for x in self.perception_data[self.perception_index+7]]

        cam1_msg = Perception2Slam()
        cam1_msg.class_list = cam1_classes
        cam1_msg.range_list = cam1_ranges
        cam1_msg.theta_list = cam1_thetas
        cam1_msg.global_index = cam1_index
        self.perception_publisher.publish(cam1_msg)

        time.sleep(0.005)

        cam2_msg = Perception2Slam()
        cam2_msg.class_list = cam2_classes
        cam2_msg.range_list = cam2_ranges
        cam2_msg.theta_list = cam2_thetas
        cam2_msg.global_index = cam2_index
        self.perception_publisher.publish(cam2_msg)


    def vectornav_callback(self):
        if self.end:return
        if self.vectornav_index == min(np.shape(self.vn200_data)[2], np.shape(self.vn300_data)[2]):
            self.end = True
            return
        try:
            msg_200 = ImuGroup()
            _accel = Vector3()
            _accel.x = self.vn200_data[0][1][self.vectornav_index]
            _accel.y = self.vn200_data[1][1][self.vectornav_index]
            _accel.z = self.vn200_data[2][1][self.vectornav_index]
            _angular = Vector3()
            _angular.z = self.vn200_data[3][1][self.vectornav_index]
            msg_200.accel = _accel
            msg_200.angularrate = _angular
            self.vn200_publisher.publish(msg_200)
            self.get_logger().info(f'---\nPublished vn_200 msg: \n {_accel}\n{_angular.z}\n----\n')
        except Exception as e:
            self.get_logger().error(f'Error during vn_200 msg creation: {repr(e)}')

        try:
            msg_300 = InsGroup()
            _status = InsStatus()
            # print(int(self.vn300_data[0][1][self.vectornav_index]))
            _status.mode = int(self.vn300_data[0][1][self.vectornav_index]) * 1
            _status.gps_heading_ins = bool(self.vn300_data[1][1][self.vectornav_index])
            _status.gps_compass = bool(self.vn300_data[2][1][self.vectornav_index])
            _vel = Vector3()
            _vel.x = self.vn300_data[3][1][self.vectornav_index]
            _vel.y = self.vn300_data[4][1][self.vectornav_index]
            _vel.z = self.vn300_data[5][1][self.vectornav_index]
            msg_300.insstatus = _status
            msg_300.velbody = _vel

            self.get_logger().info(f'---\nPublished vn_300 msg: \n {_vel}\n----\n')

            
            self.vn300_publisher.publish(msg_300)
        except Exception as e:
            self.get_logger().error(f'Error during vn_300 msg creation: {repr(e)}')

        self.vectornav_index += 1

    def sensor_callback(self):
        if self.end:return
        if self.sensor_index/8 == np.shape(self.sensor_data)[2]:
            self.end = True
            return
        try:
            msg = RxVehicleSensors()
            msg.motor_torque_actual = (self.sensor_data[0][1][int(self.sensor_index/4)])
            msg.motor_rpm = (self.sensor_data[1][1][int(self.sensor_index/4)]) * 3.9 * 9.5493 / 0.2054
            msg.brake_pressure_front = self.sensor_data[2][1][int(self.sensor_index/4)]
            msg.brake_pressure_rear = self.sensor_data[3][1][int(self.sensor_index/4)]

            self.get_logger().info(f'---\nPublished sensor msg: \n {msg.motor_rpm}\n----\n')

            self.motor_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error during sensor msg creation: {repr(e)}')
        self.sensor_index +=1

    def velocity_callback(self, msg: VelEstimation):
        meas_time = self.get_clock().now().nanoseconds/10**6
        self.log(meas_time, 0, self.vectornav_index,[msg.velocity_x, msg.velocity_y, msg.yaw_rate, msg.acceleration_x, msg.acceleration_y])

        self.results[0].append(msg.velocity_x)
        self.results[1].append(msg.velocity_y)
        self.results[2].append(msg.yaw_rate)
        self.results[3].append(msg.acceleration_x)
        self.results[4].append(msg.acceleration_y)

        for i in range(len(msg.variance_matrix)):
            self.results[i+5] = msg.variance_matrix[i]

        if self.end:
            self.get_logger().warn("Raw data depleted. Exiting...")
            # self.destroy_node()
            # rclpy.shutdown()
            exit(0)


        
    def get_data_from_file(self, file):
        data = []
        lines = file.readlines()
        for i in range(len(lines)): lines[i] = lines[i].split()
        for j in range(len(lines[0])-3):
            data.append([[],[]])     
            for line in lines: 
                try:
                    if(int(line[1])==0):
                        data[-1][0].append(float(line[0]))
                        data[-1][1].append(float(line[j+3]))
                except:
                    continue

        return data


def main(args=None):
    rclpy.init(args=args)
    time.sleep(3)
    # Spin Master Node
    p23_master_node = VelocitySimNode()
    executor = SingleThreadedExecutor()
    try:
        rclpy.spin(p23_master_node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        p23_master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()