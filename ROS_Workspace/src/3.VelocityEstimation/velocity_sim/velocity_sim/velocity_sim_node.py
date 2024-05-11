# System Related Imports
import sys
import os
import _io
import math
from pathlib import Path
import time
import numpy as np

# ROS2 Related imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory


from geometry_msgs.msg import Vector3
from custom_msgs.msg import VelEstimation, RxVehicleSensors, Perception2Slam, NodeSync
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
            ('perc_data_file', f'/home/nick/Desktop/DV-Testing-1/run_{run}/perceptionLog.txt'),
            ('sim_relative_frequency', 1)
            ]
        )

        vn200_data_filename  = self.get_parameter('vn200_data_file').get_parameter_value().string_value
        vn300_data_filename  = self.get_parameter('vn300_data_file').get_parameter_value().string_value
        sensor_data_filename = self.get_parameter('sensor_data_file').get_parameter_value().string_value
        perception_data_filename = self.get_parameter('perc_data_file').get_parameter_value().string_value
        relative_frequency = self.get_parameter('sim_relative_frequency').get_parameter_value().integer_value


        # Publisher for the synchronization topic
        self.vn200_publisher = self.create_publisher(ImuGroup, '/vn_200/raw/imu_', 10)
        self.vn300_publisher = self.create_publisher(InsGroup, '/vn_300/raw/ins_', 10)
        self.motor_publisher = self.create_publisher(RxVehicleSensors, '/canbus/sensor_data_', 10)
        self.perception_publisher = self.create_publisher(Perception2Slam, '/perception2slam_', 10)

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

        vn200_data_file.close()
        vn300_data_file.close()
        sensor_data_file.close()
        perception_data_file.close()
        
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
            msg.motor_rpm = int((self.sensor_data[1][1][int(self.sensor_index/4)]) * 3.9 * 9.5493 / 0.20540)
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

class VelocitySLAMSimNode(Node):
    def __init__(self) -> None:
        super().__init__('velocity_slam_sim_node')
        
        self.declare_parameters( namespace='',parameters=[
            ('uniform_path', True),
            ('base', ''),
            ('test', 0),
            ('run', 0),
            ('vn200_data_file', ''),
            ('vn300_data_file', ''),
            ('sensor_data_file', ''),
            ('perc_data_file', ''),
            ('perc_times_data_file', ''),
            ('velest_data_file', ''),
            ('velest_times_data_file', ''),
            ('sim_velocity', True),
            ('sim_slam', False),
            ('frequency', 1000)
        ])

        self.f_idx = {
            'vn_200': 0,
            'vn_300': 1,
            'sensor': 2,
            'perception': 3,
            'perception_times':4,
            'velest': 5,
            'velest_times':6
        }
        self.f_name = [
            'vn_200', 'vn_300', 'sensor', 'perception', 'perception_times', 'velest', 'velest_times'
        ]

        uniform_path = self.sim_slam = self.get_parameter('uniform_path').get_parameter_value().bool_value

        if uniform_path:
            base = self.get_parameter('base').get_parameter_value().string_value
            test = self.get_parameter('test').get_parameter_value().integer_value
            run = self.get_parameter('run').get_parameter_value().integer_value

            base_path = f'{base}DV-Testing-{test}/run_{run}'

            self.file_names = [
                f'{base_path}/vn200_log.txt',
                f'{base_path}/vn300_log.txt',
                f'{base_path}/canbus_sensor_log.txt',
                f'{base_path}/perceptionLog.txt',
                f'{base_path}/slam_perception_log.txt',
                f'{base_path}/velocityLog.txt',
                f'{base_path}/slam_odometry_log.txt'
            ]
        else:
            self.file_names = [
                self.get_parameter('vn200_data_file').get_parameter_value().string_value,
                self.get_parameter('vn300_data_file').get_parameter_value().string_value,
                self.get_parameter('sensor_data_file').get_parameter_value().string_value,
                self.get_parameter('perc_data_file').get_parameter_value().string_value,
                self.get_parameter('perc_times_data_file').get_parameter_value().string_value,
                self.get_parameter('velest_data_file').get_parameter_value().string_value,
                self.get_parameter('velest_times_data_file').get_parameter_value().string_value
            ]

        self.sim_velocity = self.get_parameter('sim_velocity').get_parameter_value().bool_value
        self.sim_slam = self.get_parameter('sim_slam').get_parameter_value().bool_value

        self.master_frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        self.saltas_sub = self.create_subscription(NodeSync, 'saltas_clock', self.saltas_callback, 10)


        self._publishers = [
            self.create_publisher(ImuGroup, '/vn_200/raw/imu', 10),
            self.create_publisher(InsGroup, '/vn_300/raw/ins', 10),
            self.create_publisher(RxVehicleSensors, '/canbus/sensor_data', 10),
            self.create_publisher(Perception2Slam, '/perception2slam', 10),
            self.create_publisher(VelEstimation, '/velocity_estimation', 10)
        ]

        self.master_timer = self.create_timer(1/self.master_frequency, self.master_callback)

        self.data_files: list[_io.TextIOWrapper] = []
        self.data_files_check:list[bool] = []

        for filename in self.file_names:
            try:
                tmp = open(filename, 'r')
            except:
                self.data_files_check.append(False)
            else:
                self.data_files_check.append(True)
                self.data_files.append(tmp) 

        self.data: list[list[Entry]] = []

        for i in range(len(self.data_files)-4):
            self.data.append(self.get_data_from_file(self.data_files[i], self.f_name[i]) if self.data_files_check[i] else None)

        self.perception_data = [line.split() for line in self.data_files[self.f_idx['perception']].readlines()] if self.data_files_check[self.f_idx['perception']] else None

        self.perception_data_times = [line.split() for line in self.data_files[self.f_idx['perception_times']].readlines()]if self.data_files_check[self.f_idx['perception_times']] else None

        if self.perception_data is not None and self.perception_data_times is not None:
            self.data.append(self.create_perc_entries())
        else:
            self.data.append(None)

        self.velest_data = [line.split() for line in self.data_files[self.f_idx['velest']].readlines()] if self.data_files_check[self.f_idx['velest']] else None

        self.velest_data_times = [line.split() for line in self.data_files[self.f_idx['velest_times']].readlines()]if self.data_files_check[self.f_idx['velest_times']] else None

        if self.velest_data is not None and self.velest_data_times is not None:
            self.data.append(self.create_vel_entries())
        else:
            self.data.append(None)


        print(len(self.data))

        for item in self.f_idx:
            if item == 'perception_times' or item == 'velest_times':continue
            idx = self.f_idx[item] if item!='velest' else 4

            if(self.data[idx] is not None and len(self.data[idx])!=0):
                self.get_logger().info(f'{item} dataset is ok. Length is {len(self.data[idx])}')
            else:
                self.get_logger().warn(f'{item} dataset is not ok.')

        for file in self.data_files:
            file.close()


        self.global_index = 0
        self.read_index = [0, 0, 0, 0, 0]
        self.max_idxs = [len(x)-1 for x in self.data]

        _t0s = [x[0].get_time() for x in self.data]
        self.t0 = max(_t0s)
        self.time = self.t0
        self.dt = 1000 / self.master_frequency
        self.finish = False

    def master_callback(self):
        # if(self.time - self.t0>200):exit(0)
        if self.finish:
            exit(0)
        lst = []
        if self.sim_velocity:
            lst = [0, 1, 2]
        else:
            lst = [4]
        if self.sim_slam:
            lst.append(3)
        for i in lst:
            # print(self.time, self.data[i][self.read_index[i]].get_time(math.log10(self.master_frequency)-3))
            if self.time == self.data[i][self.read_index[i]].get_time(math.log10(self.master_frequency)-3):
                msg = self.data[i][self.read_index[i]]._msg
                if i==3:
                    msg.global_index
                self._publishers[i].publish(self.data[i][self.read_index[i]]._msg)
                self.get_logger().info(f'--- \nPublished {self.f_name[i]} msg. \n----')
                self.read_index[i]+=1
                if self.read_index[i]==self.max_idxs[i]:
                    self.finish = True
            else:
                while self.time>self.data[i][self.read_index[i]].get_time(math.log10(self.master_frequency)-3):
                    self.read_index[i]+=1
                if self.time == self.data[i][self.read_index[i]].get_time(math.log10(self.master_frequency)-3):
                    self._publishers[i].publish(self.data[i][self.read_index[i]]._msg)
                    self.get_logger().info(f'--- \nPublished {self.f_name[i]} msg. \n----')
                    self.read_index[i]+=1
                    if self.read_index[i]==self.max_idxs[i]:
                        self.finish = True
        self.time += self.dt
        self.time = round(self.time, int(math.log10(self.master_frequency)-3))
        return
        
    def saltas_callback(self, msg:NodeSync) -> None:
        self.global_index = msg.global_index if msg.exec_perception else self.global_index

    def create_perc_entries(self):
        data:list[Entry] = []
        l_d = len(self.perception_data)
        l_t = len(self.perception_data_times)

        # print(f' Perc lens is {l_d}, {l_t}')

        # for i in range(l_t-1):
        #     t, g_idx_t = float(self.perception_data_times[i][0]), int(self.perception_data_times[i][2])
        #     if(4*i+3>=l_d-1):
        #         break
        #     g_idx_d = int(self.perception_data[4*i][0])
        #     classes = [int(x) for x in self.perception_data[4*i+1]]
        #     ranges = [float(x) for x in self.perception_data[4*i+2]]
        #     thetas = [float(x) for x in self.perception_data[4*i+3]]
        #     if g_idx_d != g_idx_t:
        #         print(g_idx_d, g_idx_t)
        #         continue

        #     data.append(Entry(time = t, _data = [g_idx_d, classes, ranges, thetas], type = 'perception'))

        # return data
        for i in range(int(l_d/4)-1):
            g_idx_d = int(self.perception_data[4*i][0])
            classes = [int(x) for x in self.perception_data[4*i+1]]
            ranges = [float(x) for x in self.perception_data[4*i+2]]
            thetas = [float(x) for x in self.perception_data[4*i+3]]

            done = False

            for j in range(len(self.perception_data_times)-1):
                # print(self.perception_data_times[j][2] , g_idx_d, end='\t||\t')
                if int(self.perception_data_times[j][2]) == int(g_idx_d):
                    data.append(Entry(time = float(self.perception_data_times[j][0]), _data = [g_idx_d, classes, ranges, thetas], type = 'perception'))
                    self.perception_data_times.pop(j)
                    done = True
                    break
        return data

    

    
    def create_vel_entries(self):
        data:list[Entry] = []
        l_d = len(self.velest_data)
        l_t = len(self.velest_data_times)

        for i in range(int(l_t/2-1)):
            t, g_idx_t = float(self.velest_data_times[2*i][0]), int(self.velest_data_times[2*i][2])
            if(5*i+4>=l_d-1):
                break
            g_idx_d = int(self.velest_data[5*i][0])
            vx = [float(x) for x in self.velest_data[5*i+1]][0]
            vy = [float(x) for x in self.velest_data[5*i+2]][0]
            r = [float(x) for x in self.velest_data[5*i+3]][0]
            covs = [float(x) for x in self.velest_data[5*i+4]]
            if g_idx_d != g_idx_t:
                print(g_idx_d, g_idx_t, end='---')
                continue

            data.append(Entry(time = t, _data = [g_idx_d, vx, vy, r, covs], type = 'velest'))

        return data
    

    def get_data_from_file(self, file, type):
        data: list[Entry] = []
        lines = file.readlines()
        for i in range(len(lines)): lines[i] = lines[i].split()
        # for j in range(len(lines[0])-3):
        #     for line in lines: 
        #         try:
        #             if(int(line[1])==0):
        #                 # data.append(Entry(float(line[0]), float(line[j+3])))
        #                 data.append(Entry(float(line[0]), [float(x) for x in line[j+3:]]))
        #         except:
        #             continue
        
        for line in lines:
            if(len(line)!=len(lines[0])):
                continue
            if(int(line[1])==1):
                continue
            # if type=='sensor':
            #     print(line[1])
            data.append(Entry(float(line[0]), [float(x) for x in line[3:]], type))

        return data

        

class Entry:
    timestamp: float = 0
    data: list[float] = []
    type: str = ''

    def __init__(self, time: float, _data: list[float], type:str = '')->None:
        self.timestamp = time
        self.data = _data
        self.type = type
        if type!='':
            self.create_msg()

    def get_time(self, accuracy:int = 0)->float:
        # print(accuracy)
        return round(self.timestamp, int(accuracy))
    
    def create_msg(self) -> None:
        if self.type == 'vn_200':
            if len(self.data)!=4:
                raise ValueError(f'Incorrect data length: Wanted 4, got {len(self.data)}')
            self._msg = ImuGroup()
            self._msg.accel.x = self.data[0]
            self._msg.accel.y = self.data[1]
            self._msg.accel.z = self.data[2]
            self._msg.angularrate.z = self.data[3]

        elif self.type == 'vn_300':
            if len(self.data)!=7:
                raise ValueError(f'Incorrect data length: Wanted 6, got {len(self.data)}')
            self._msg = InsGroup()
            self._msg.insstatus = InsStatus()
            self._msg.insstatus.mode = int(self.data[0])
            self._msg.insstatus.gps_heading_ins = bool(self.data[1])
            self._msg.insstatus.gps_compass = bool(self.data[2])
            self._msg.velbody.x = self.data[3]
            self._msg.velbody.y = self.data[4]
            self._msg.velbody.z = self.data[5]

        elif self.type == 'sensor':
            if len(self.data)!=5:
                raise ValueError(f'Incorrect data length: Wanted 4, got {len(self.data)}')
            self._msg = RxVehicleSensors()
            self._msg.motor_torque_actual = self.data[0]
            self._msg.motor_rpm = int(self.data[1] / (0.2 / (9.5493*3.9)))
            self._msg.brake_pressure_front = self.data[2]
            self._msg.brake_pressure_rear = self.data[3]

        elif self.type=='perception':
            if len(self.data)!=4:
                raise ValueError(f'Incorrect data length: Wanted 4, got {len(self.data)}')
            self._msg = Perception2Slam()
            self._msg.global_index = self.data[0]
            self._msg.class_list = self.data[1]
            self._msg.range_list = self.data[2]
            self._msg.theta_list = self.data[3]

        elif self.type=='velest':
            if len(self.data)!=5:
                raise ValueError(f'Incorrect data length: Wanted 5, got {len(self.data)}')
            self._msg = VelEstimation()
            self._msg.global_index = self.data[0]
            self._msg.velocity_x = self.data[1]
            self._msg.velocity_y = self.data[2]
            self._msg.yaw_rate = self.data[3]
            self._msg.variance_matrix = self.data[4]


def main(args=None):
    rclpy.init(args=args)
    # Spin Master Node
    sim_node2 = VelocitySLAMSimNode()
    # time.sleep(1)
    # sim_node = VelocitySimNode()

    # executor = SingleThreadedExecutor()
    executor2 = SingleThreadedExecutor()
    try:
        # rclpy.spin(sim_node, executor)
        rclpy.spin(sim_node2, executor2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sim_node2.destroy_node()
        # sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()